These are 7 files you will need to help you to invert, transpose and multiply matrices.

The easiest way to get them working is to add them to one of the existing directory for example "Library". Open the Jamfile in that directory, add the cpp files' filename to NAMES. You need to leave a space before ";"

If you notice, some of the cpp files are actually old style c code, I did a quick modification to some of them to make them work with Jam and our current viewer code. If you want the old version of them, you can get them from the "old" directory.

Ming