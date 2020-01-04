# Makegen - Makefile Generator
# Version: 2.0
# (C) Copyright 2010 - 2018 Pavel Surynek
# http://www.surynek.com
# pavel@surynek.com

import fileinput
import string
import shutil
import os
import sys
import stat

class ModuleRecord:
  pass

def print_intro():
  print "Makegen 2.0 - Makefile Generator"
  print "(C) Copyright 2010 - 2018 Pavel Surynek"
  print "---------------------------------------"

include_dirs = set()
program_dirs = set()
project_dirs = list()
header_dirs = list()
modules = list()
selected_modules = list()
web_files = list()
art_files = list()

def get_first_word(line):
  begin = -1
  end = -1
  for i in range(len(line)):
    if begin == -1:
      if line[i] not in string.whitespace:
        begin = i
    else:
      if end == -1:
        if line[i] in string.whitespace:
          end = i
          break
  if begin == -1:
    return ""
  else:
    if end == -1:
      end = len(line)
  return line[begin:end]


def get_remaining_words(line):
  begin = -1
  end = -1
  for i in range(len(line)):
    if begin == -1:
      if line[i] not in string.whitespace:
        begin = i
    else:
      if end == -1:
        if line[i] in string.whitespace:
          end = i
      else:
        if line[i] not in string.whitespace:
          end = i
          break
  if begin == -1 or end == -1:
    return ""
  return line[end:len(line)]

              
def load_modules(modules_file):
  while True:
    module_record = ModuleRecord()
    module_identifier = modules_file.readline()
    
    while module_identifier != "" and get_first_word(module_identifier) == "":
      module_identifier = modules_file.readline()
      
    if module_identifier == "":
      break

    module_record.identifier = get_first_word(module_identifier)

    module_type = modules_file.readline()

    if string.find(module_type, "program") == 0:
      module_record.type = "program"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)
      if module_record.directory not in program_dirs:
        program_dirs.add(module_record.directory)
        project_dirs.append(module_record.directory)

      module_headers = modules_file.readline()
      module_record.headers = list()

      header = get_first_word(module_headers)
      while header != "":
        module_headers = get_remaining_words(module_headers)
        module_record.headers.append(header)
        if header[0] == '/':
          head_dir = construct_header_directory(header)
          include_dirs.add(head_dir)
          if head_dir not in project_dirs:
            header_dirs.append(head_dir)
        else:
          include_dirs.add(module_record.directory)
          if module_record.directory not in project_dirs:
            project_dirs.append(module_record.directory)
        header = get_first_word(module_headers)
      
      module_sources = modules_file.readline()
      module_record.sources = list()

      source = get_first_word(module_sources)
      while source != "":
        module_sources = get_remaining_words(module_sources)
        module_record.sources.append(source)
        source = get_first_word(module_sources)      

    elif string.find(module_type, "executables") == 0:
      module_record.type = "executables"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)

      module_files = modules_file.readline()
      module_record.files = list()

      file = get_first_word(module_files)
      while file != "":
        module_files = get_remaining_words(module_files)
        module_record.files.append(file)
        file = get_first_word(module_files)

    elif string.find(module_type, "application") == 0:
      module_record.type = "application"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)
      program_dirs.add(module_record.directory)
      if module_record.directory not in project_dirs:
        project_dirs.append(module_record.directory)

      module_headers = modules_file.readline()
      module_record.headers = list()

      header = get_first_word(module_headers)
      while header != "":
        module_headers = get_remaining_words(module_headers)
        module_record.headers.append(header)
        if header[0] == '/':
          head_dir = construct_header_directory(header)
          include_dirs.add(head_dir)
          if head_dir not in project_dirs:
            header_dirs.append(head_dir)
        else:
          include_dirs.add(module_record.directory)
          if module_record.directory not in project_dirs:
            project_dirs.append(module_record.directory)
        header = get_first_word(module_headers)
      
      module_sources = modules_file.readline()
      module_record.sources = list()

      source = get_first_word(module_sources)
      while source != "":
        module_sources = get_remaining_words(module_sources)
        module_record.sources.append(source)
        source = get_first_word(module_sources)

      module_libraries = modules_file.readline()
      module_record.libraries = list()

      library = get_first_word(module_libraries)
      while library != "":
        module_libraries = get_remaining_words(module_libraries)
        module_record.libraries.append(library)
        library = get_first_word(module_libraries)

      module_own_libraries = modules_file.readline()
      module_record.own_libraries = list()

      own_library = get_first_word(module_own_libraries)
      while own_library != "":
        module_own_libraries = get_remaining_words(module_own_libraries)
        module_record.own_libraries.append(own_library)
        own_library = get_first_word(module_own_libraries)

    elif string.find(module_type, "library") == 0:
      module_record.type = "library"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)
      program_dirs.add(module_record.directory)
      if module_record.directory not in project_dirs:
        project_dirs.append(module_record.directory)

      module_headers = modules_file.readline()
      module_record.headers = list()

      header = get_first_word(module_headers)
      while header != "":
        module_headers = get_remaining_words(module_headers)
        module_record.headers.append(header)
        if header[0] == '/':
          head_dir = construct_header_directory(header)
          include_dirs.add(head_dir)
          if head_dir not in project_dirs:
            header_dirs.append(head_dir)
        else:
          include_dirs.add(module_record.directory)
          if module_record.directory not in project_dirs:
            project_dirs.append(module_record.directory)
        header = get_first_word(module_headers)
      
      module_sources = modules_file.readline()
      module_record.sources = list()

      source = get_first_word(module_sources)
      while source != "":
        module_sources = get_remaining_words(module_sources)
        module_record.sources.append(source)
        source = get_first_word(module_sources)

      module_own_libraries = modules_file.readline()
      module_record.own_libraries = list()

      own_library = get_first_word(module_own_libraries)
      while own_library != "":
        module_own_libraries = get_remaining_words(module_own_libraries)
        module_record.own_libraries.append(own_library)
        own_library = get_first_word(module_own_libraries)
      
    elif string.find(module_type, "files") == 0:
      module_record.type = "files"
      module_directory = modules_file.readline()
      module_record.directory = get_first_word(module_directory)

      if module_record.directory not in program_dirs:
        project_dirs.append(module_record.directory)

      module_files = modules_file.readline()
      module_record.files = list()

      file = get_first_word(module_files)
      while file != "":
        module_files = get_remaining_words(module_files)
        module_record.files.append(file)

        if (   (len(file) >= 4 and string.find(file, ".php") == len(file) - 4)
            or (len(file) >= 5 and string.find(file, ".html") == len(file) - 5)
            or (len(file) >= 3 and string.find(file, ".js") == len(file) - 3)
            or (len(file) >= 4 and string.find(file, ".css") == len(file) - 4)):
          web_files.append(module_record.directory[1:len(module_record.directory)] + "/" + file);

        if (len(file) >= 4 and string.find(file, ".png") == len(file) - 4) or (len(file) >= 4 and string.find(file, ".jpg") == len(file) - 4) or (len(file) >= 4 and string.find(file, ".ico") == len(file) - 4):
          art_files.append(module_record.directory[1:len(module_record.directory)] + "/" + file);

        file = get_first_word(module_files)
       
    else:
      print "Error: Unrecognized module type: " + module_type
      break;

    modules.append(module_record)

  for hd in header_dirs:
    if hd not in project_dirs:
      project_dirs.append(hd)


def load_selection(selection_file):
  for line in selection_file:
    module = get_first_word(line)
    selected_modules.append(module)

  for md in modules:
    sel = False
    for smd in selected_modules:
      if smd == md.identifier:
        sel = True
        break
    md.selected = sel


def construct_object_name_debug(module_directory, source):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if source[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    ext_pos = string.find(source, ".")
    return prefix + source[1:ext_pos] + ".o_dbg"
  else:
    ext_pos = string.find(source, ".")
    return source[0:ext_pos] + ".o_dbg"


def construct_object_name_optimized(module_directory, source):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if source[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    ext_pos = string.find(source, ".")
    return prefix + source[1:ext_pos] + ".o_opt"
  else:
    ext_pos = string.find(source, ".")
    return source[0:ext_pos] + ".o_opt"


def construct_header_name(module_directory, header):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if header[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    return prefix + header[1:len(header)]
  else:
    return header[0:len(header)]


def construct_header_directory(header):
  start_hdr = 0
  pos_hdr = string.find(header, "/", start_hdr)
  while pos_hdr >= 0:
    start_hdr = pos_hdr + 1
    pos_hdr = string.find(header, "/", start_hdr)
  return header[0:start_hdr-1]


def construct_source_name(module_directory, source):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)

  if source[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    return prefix + source[1:len(source)]
  else:
    return source[0:len(source)]


def construct_library_name_debug(module_directory, library):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)
  
  if library[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    return prefix + library[1:len(library)] + "_dbg"
  else:
    return library[0:len(library)] + "_dbg"


def construct_library_name_optimized(module_directory, library):
  depth = 0
  start = 0

  pos = string.find(module_directory, "/", start)
  while pos >= 0:
    depth = depth + 1
    start = pos + 1
    pos = string.find(module_directory, "/", start)
  
  if library[0] == '/':
    prefix = ""
    for d in range(depth):
      prefix = prefix + "../"
    return prefix + library[1:len(library)] + "_opt"
  else:
    return library[0:len(library)] + "_opt"


def construct_library_directory(library):
  start_hdr = 0
  pos_hdr = string.find(library, "/", start_hdr)
  while pos_hdr >= 0:
    start_hdr = pos_hdr + 1
    pos_hdr = string.find(library, "/", start_hdr)
  return library[0:start_hdr-1]


def construct_library_module_name(library):
  start_lib = 0
  pos_lib = string.find(library, "/", start_lib)
  while pos_lib >= 0:
    start_lib = pos_lib + 1
    pos_lib = string.find(library, "/", start_lib)
  return library[start_lib:len(library)]


def construct_library_module_name_debug(library):
  start_lib = 0
  pos_lib = string.find(library, "/", start_lib)
  while pos_lib >= 0:
    start_lib = pos_lib + 1
    pos_lib = string.find(library, "/", start_lib)
  return library[start_lib:len(library)] + "_dbg"


def construct_library_module_name_optimized(library):
  start_lib = 0
  pos_lib = string.find(library, "/", start_lib)
  while pos_lib >= 0:
    start_lib = pos_lib + 1
    pos_lib = string.find(library, "/", start_lib)
  return library[start_lib:len(library)] + "_opt"


def collect_library_header_directories(directory, library_module):
  header_dirs = set()
  for hd in library_module.headers:
    if hd[0] == '/':
      header = hd
    else:
      header = library_module.directory + "/" + hd
    hdr = construct_header_name(directory, header)
    hdr_dir = construct_header_directory(hdr)
    header_dirs.add(hdr_dir)
  for own in library_module.own_libraries:
    for module in modules:
      if module.identifier == construct_library_module_name(own):
        sub_header_dirs = collect_library_header_directories(directory, module)
        for sub_head in sub_header_dirs:
          header_dirs.add(sub_head)
  return header_dirs


def generate_rules(clean_pattern):
 for pd in project_dirs:
   if pd[0] == '/':
     pd2 = pd[1:len(pd)]

   if len(pd2) == 0:
     continue

   if os.path.isdir(pd2):
     print "Processing existent directory ... " + pd2
   else:
     print "Processing non-existent directory ... " + pd2
     os.makedirs(pd2)
   print "  Generating '" + pd2 + "/Makefile' ... ",
   makefile = open(pd2 + "/Makefile", "w")

   create_rules = False
   for md in modules:
     if md.selected and (md.type == "program" or md.type == "application" or md.type == "library"):
       if pd == md.directory:
         create_rules = True

   if create_rules:
     makefile.write("all: debug\n")
     makefile.write("\n")
     makefile.write("debug:\t")

     objects = set()

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             objects.add(obj)
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             objects.add(obj)
       if md.selected and md.type == "library":
         if pd == md.directory:
           obj = "lib" + md.identifier + "_dbg.a"
           objects.add(obj)
     for obj in objects:
       makefile.write(obj + " ")
     makefile.write("\n")

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           makefile.write("\tg++ -o " + md.identifier)
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             makefile.write(obj + " ")
           makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           makefile.write("\tg++ ")
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             makefile.write(obj + " ")
           lib_dirs = set()
           for lbo in md.own_libraries:
             lib_name = construct_library_name_debug(pd, lbo)
             lib_dir = construct_library_directory(lib_name)
             lib_dirs.add(lib_dir)
           for lbd in lib_dirs:
             makefile.write("-L" + lbd + " ")
           for lbo in md.own_libraries:
             lib_name = construct_library_name_debug(pd, lbo)
             lib_dir = construct_library_directory(lib_name)
             lib_module = construct_library_module_name_debug(lbo)
             library = "-l" + lib_module + " "
             makefile.write(library)
           for lb in md.libraries:
             if lb == "pthread":
               library = "-" + lb + " "
             else:
               library = "-l" + lb + " "
             makefile.write(library)
           makefile.write("-o" + md.identifier)
           makefile.write("\n")
     makefile.write("\n")

     makefile.write("optimized:\t")

     objects = set()

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             objects.add(obj)
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             objects.add(obj)
       if md.selected and md.type == "library":
         if pd == md.directory:
           obj = "lib" + md.identifier + "_opt.a"
           objects.add(obj)

     for obj in objects:
       makefile.write(obj + " ")
     makefile.write("\n")

     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           makefile.write("\tg++ -o" + md.identifier)
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           makefile.write("\tg++ ")
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             makefile.write(obj + " ")
           lib_dirs = set()
           for lbo in md.own_libraries:
             lib_name = construct_library_name_optimized(pd, lbo)
             lib_dir = construct_library_directory(lib_name)
             lib_dirs.add(lib_dir)
           for lbd in lib_dirs:
             makefile.write("-L" + lbd + " ")
           for lbo in md.own_libraries:
             lib_name = construct_library_name_optimized(pd, lbo)
             lib_dir = construct_library_directory(lib_name)
             lib_module = construct_library_module_name_optimized(lbo)
             library = "-l" + lib_module + " "
             makefile.write(library)
           for lb in md.libraries:
             if lb == "pthread":
               library = "-" + lb + " "
             else:
               library = "-l" + lb + " "
             makefile.write(library)
           makefile.write("-o" + md.identifier)
           makefile.write("\n")
     makefile.write("\n")

     finished = set();
     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               header_dirs = set()
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
                 if hdr[0] == '.':
                   hdr_dir = construct_header_directory(hdr)
                   header_dirs.add(hdr_dir)
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -std=c++0x -g -c ")
               for hdr_dir in header_dirs:
                 makefile.write("-I" + hdr_dir + " ")
               makefile.write("-o" + obj + " " + src + "\n")
               makefile.write("\n")
       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               header_dirs = set()
               for lbo in md.own_libraries:
                 for lbo_mod in modules:
                   lbo_mod_name = construct_library_module_name(lbo)
                   if lbo_mod_name == lbo_mod.identifier:                     
                     for lbo_hdr in lbo_mod.headers:                       
                       if lbo_hdr[0] == '/':
                         lbo_header = lbo_hdr
                       else:
                         lbo_header = lbo_mod.directory + "/" + lbo_hdr
                       lbo_header_name = construct_header_name(md.directory, lbo_header)
                       makefile.write(lbo_header_name + " ")
                       if lbo_header_name[0] == '.':
                         lbo_hdr_dir = construct_header_directory(lbo_header_name)
                         header_dirs.add(lbo_hdr_dir)
                     break
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
                 if hdr[0] == '.':
                   hdr_dir = construct_header_directory(hdr)
                   header_dirs.add(hdr_dir)
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")

               makefile.write("\tg++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c ")
               for hdr_dir in header_dirs:
                 makefile.write("-I" + hdr_dir + " ")
               makefile.write("-o" + obj + " " + src + "\n")
               makefile.write("\n")

       if md.selected and md.type == "library":
         if pd == md.directory:
           head_dirs = collect_library_header_directories(md.directory, md)
           makefile.write("lib" + md.identifier + "_dbg.a:\t")
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             makefile.write(obj + " ")             
           makefile.write("\n\tar r " + "lib" + md.identifier + "_dbg.a")
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n");
           makefile.write("\n");
           for sr in md.sources:
             obj = construct_object_name_debug(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")

               makefile.write("\tg++ -Wall -Wextra -pedantic -Wno-class-memaccess -Wno-long-long -Wno-unused-result -Wno-sign-compare -Wno-delete-non-virtual-dtor -g -std=c++0x -c ")
               for hdr_dir in head_dirs:
                 makefile.write("-I" + hdr_dir + " ")
               makefile.write("-o" + obj + " " + src + "\n")
               makefile.write("\n")
     makefile.write("\n")
     
     finished = set();
     for md in modules:
       if md.selected and md.type == "program":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               header_dirs = set()
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
                 if hdr[0] == '.':
                   hdr_dir = construct_header_directory(hdr)
                   header_dirs.add(hdr_dir)
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ -std=c++0x -c -w -O9 -mtune=native ")
               for hdr_dir in header_dirs:
                 makefile.write("-I" + hdr_dir + " ")
               makefile.write("-o" + obj + " " + src + "\n")
               makefile.write("\n")

       if md.selected and md.type == "application":
         if pd == md.directory:
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               header_dirs = set()
               for lbo in md.own_libraries:
                 for lbo_mod in modules:
                   lbo_mod_name = construct_library_module_name(lbo)
                   if lbo_mod_name == lbo_mod.identifier:
                     for lbo_hdr in lbo_mod.headers:                       
                       if lbo_hdr[0] == '/':
                         lbo_header = lbo_hdr
                       else:
                         lbo_header = lbo_mod.directory + "/" + lbo_hdr
                       lbo_header_name = construct_header_name(md.directory, lbo_header)
                       makefile.write(lbo_header_name + " ")
                       if lbo_header_name[0] == '.':
                         lbo_hdr_dir = construct_header_directory(lbo_header_name)
                         header_dirs.add(lbo_hdr_dir)
                     break
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
                 if hdr[0] == '.':
                   hdr_dir = construct_header_directory(hdr)
                   header_dirs.add(hdr_dir)
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")

               makefile.write("\tg++ -std=c++0x -c -w -O9 -mtune=native ")
               for hdr_dir in header_dirs:
                 makefile.write("-I" + hdr_dir + " ")
               makefile.write("-o" + obj + " " + src + "\n")
               makefile.write("\n")

       if md.selected and md.type == "library":
         if pd == md.directory:
           head_dirs = collect_library_header_directories(md.directory, md)
           makefile.write("lib" + md.identifier + "_opt.a:\t")
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             makefile.write(obj + " ")             
           makefile.write("\n\tar r " + "lib" + md.identifier + "_opt.a")
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             makefile.write(" " + obj)
           makefile.write("\n");
           makefile.write("\n");
           for sr in md.sources:
             obj = construct_object_name_optimized(md.directory, sr)
             if obj not in finished:
               finished.add(obj)
               makefile.write(obj + ":\t")
               for hd in md.headers:
                 hdr = construct_header_name(md.directory, hd)
                 makefile.write(hdr + " ")
               src = construct_source_name(md.directory, sr)
               makefile.write(src + "\n")
               makefile.write("\tg++ -std=c++0x -c -w -O9 -mtune=native ")
               for hdr_dir in head_dirs:
                 makefile.write("-I" + hdr_dir + " ")
               makefile.write("-o" + obj + " " + src + "\n")
               makefile.write("\n")
     makefile.write("\n")
   else:
     makefile.write("all: debug\n\n")
     makefile.write("debug:\n\n")
     makefile.write("optimized:\n\n")

   makefile.write("clean:\n")
   makefile.write("\trm -f -r " + clean_pattern)
   for md in modules:
     if md.selected and (md.type == "program" or md.type == "application"):
       if pd == md.directory:
         makefile.write(" " + md.identifier)
            
   makefile.write("\n")
   makefile.close()
   print "OK"

 print "  Generating 'Makefile' ... ",
 main_makefile = open("Makefile", "w")
 main_makefile.write("SUBDIRS =")
 for pd in project_dirs:
   main_makefile.write(" " + pd[1:len(pd)])
 main_makefile.write("\n\n")
 main_makefile.write("all: debug\n")
 main_makefile.write("\n")
 main_makefile.write("debug:\n")
 main_makefile.write("\tfor dir in $(SUBDIRS); do make -C $$dir debug; done\n")
 main_makefile.write("\n")
 main_makefile.write("optimized:\n")
 main_makefile.write("\tfor dir in $(SUBDIRS); do make -C $$dir optimized; done\n")  
 main_makefile.write("\n")

 main_makefile.write("release:\toptimized\n")
 for md in modules:
   if md.selected and (md.type == "executables"):
     for fl in md.files:
       main_makefile.write("\tcp " + md.directory[1:len(md.directory)] + "/" + fl + " bin\n")  

 main_makefile.write("download:\toptimized\n")
 for md in modules:
   if md.selected and (md.type == "executables"):
     for fl in md.files:
       main_makefile.write("\tcp " + md.directory[1:len(md.directory)] + "/" + fl + " bin\n")  

 main_makefile.write("\n")
 main_makefile.write("clean:\n")
 main_makefile.write("\tfor dir in $(SUBDIRS); do make -C $$dir clean; done\n")
 main_makefile.write("\trm -f " + clean_pattern + "\n")
 for md in modules:
   if md.selected and (md.type == "executables"):
     for fl in md.files:
       main_makefile.write("\trm -f bin/" + fl + "\n")  
  
 main_makefile.close()

 deploy_file = open("deploy.sh", "w")
 deploy_file.write("echo \"Deploying web files ...\"\n");
 for wfile in web_files:
   deploy_file.write("echo -n \"  Deploying " + wfile + " ... \"\n");
   deploy_file.write("cp " + wfile + " /var/www/html");
   deploy_file.write("\n");
   deploy_file.write("echo \"OK\"\n");
 deploy_file.write("\nmkdir -p /var/www/html/art\n");

 deploy_file.write("echo \"Deploying art files ...\"\n");
 for afile in art_files:
   deploy_file.write("echo -n \"  Deploying " + afile + " ... \"\n");
   deploy_file.write("cp " + afile + " /var/www/html/art");
   deploy_file.write("\n");
   deploy_file.write("echo \"OK\"\n");
 deploy_file.close();
 os.chmod("deploy.sh", stat.S_IRWXU | stat.S_IXGRP | stat.S_IXOTH);

 print "OK"

print_intro()

modules_file = open("modules", "r")
load_modules(modules_file)
modules_file.close()

selection_file = open(sys.argv[1], "r")
load_selection(selection_file)
selection_file.close()

clean_file = open("clear", "r")
clean_pattern = clean_file.readline()
clean_file.close()

generate_rules(clean_pattern)
print "."
