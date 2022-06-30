"""
    Searches recursive all files "*.h" and "*.hpp" and adds line "include \"sick_scan/sick_scan_base.h\" at the beginning of all files.

    Usage:
    python3 add_sick_scan_base_header.py
    
"""

import os

# List and return all files in a folder recursively, see https://stackoverflow.com/questions/18394147/how-to-do-a-recursive-sub-folder-search-and-return-files-in-a-list
def scandirectory(dir, extensions, excludes):
    subfolders, files = [], []
    for f in os.scandir(dir):
        is_excluded = False
        for exclude in excludes:
            if exclude in f.path:
                is_excluded = True
                break
        if is_excluded:
            continue
        if f.is_dir():
            subfolders.append(f.path.replace("\\","/"))
        if f.is_file():
            if os.path.splitext(f.name)[1].lower() in extensions:
                files.append(f.path.replace("\\","/"))
    for dir in list(subfolders):
        f = scandirectory(dir, extensions, excludes)
        files.extend(f)
    return files

# Run this script
if __name__ == '__main__':
    
    folder = "../.." # search all files "*.h" and "*.hpp" in <folder>
    exclude_string = "#include \"sick_scan/sick_scan_base.h\"" # keep file untouched, if it starts with <exclude_string>
    additional_line = "#include \"sick_scan/sick_scan_base.h\" /* Base definitions included in all header files, added by add_sick_scan_base_header.py. Do not edit this line. */\n" # add this line at the beginning of all files

    header_files = scandirectory(folder, [".h", ".hpp"], ["sick_scan_base.h", "sick_scan_api.h"])
    for file in header_files:
        content = ""
        with open(file, "r") as f:
            content = f.read()
        if content != "" and not content.startswith(exclude_string):
            with open(file, "w") as f:
                f.write(additional_line)
                f.write(content)
                print("{}: added \"{} ...\"".format(file, additional_line[:37]))
        else:
                print("{}: skipped".format(file))
    print("add_sick_scan_base_header.py finished.")
    