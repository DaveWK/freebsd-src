# SPDX-License-Identifier: BSD-2-Clause
# Copyright (c) 2026 Dave Klotz

atf_test_case replace_short_entry cleanup
replace_short_entry_head()
{
	atf_set "descr" "Replacing a short-only name must preserve adjacent entries"
	atf_set "require.user" "root"
	atf_set "require.progs" "mdconfig newfs_msdos mount umount truncate"
}
replace_short_entry_body()
{
	atf_check mkdir mnt
	atf_check truncate -s 64m disk.img
	atf_check -o save:md mdconfig -a -t vnode -f "$(pwd)/disk.img"
	md=$(cat md)
	atf_check -o ignore -e ignore newfs_msdos -F 16 "/dev/$md"
	atf_check mount -t msdosfs -o longnames "/dev/$md" "$(pwd)/mnt"

	# EFI and U-BOOT.ITB each occupy one short directory entry.
	atf_check mkdir mnt/EFI
	printf 'loader sentinel\n' > loader
	atf_check cp loader mnt/EFI/LOADER.EFI
	printf 'old payload\n' > mnt/U-BOOT.ITB
	atf_check umount "$(pwd)/mnt"
	atf_check mount -t msdosfs -o longnames "/dev/$md" "$(pwd)/mnt"

	printf 'replacement payload\n' > expected
	atf_check cp expected mnt/u-boot.itb.temporary
	# Lower case requires an additional long filename entry.
	atf_check mv -f mnt/u-boot.itb.temporary mnt/u-boot.itb
	atf_check cmp expected mnt/u-boot.itb
	atf_check umount "$(pwd)/mnt"
	atf_check mount -t msdosfs -o longnames "/dev/$md" "$(pwd)/mnt"

	# Remount so cached directory vnodes cannot hide on-disk corruption.
	atf_check test -d mnt/EFI
	atf_check cmp loader mnt/EFI/LOADER.EFI
	atf_check cmp expected mnt/u-boot.itb
}
replace_short_entry_cleanup()
{
	if [ -f md ]; then
		umount "$(pwd)/mnt" 2>/dev/null || :
		mdconfig -d -u "$(cat md)"
	fi
}

atf_init_test_cases()
{
	atf_add_test_case replace_short_entry
}
