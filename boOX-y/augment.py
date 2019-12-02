# Augment - C++ Source Code Augmenter
# Version: 2.2
# (C) Copyright 2010-2019 Pavel Surynek
# http://www.surynek.com
# pavel@surynek.com

import fileinput
import string
import shutil
import os

def print_header_frame(file, width, height):
  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")

  for i in range(height-2):
    file.write("/*")
    for j in range(width-4):
      file.write(" ")
    file.write("*/")
    file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")


def print_header_info_frame(file, width, height, product, product_line, copyright, copyright_line, email, email_line, filename):
  file.write("/*")
  for j in range(width-4):
    file.write("=")
  file.write("*/")
  file.write("\n")

  for i in range(height-2):
    if i == product_line:
      left = (width - len(product) - 4) / 2
      right = width - len(product) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(product)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == copyright_line:
      left = (width - len(copyright) - 4) / 2
      right = width - len(copyright) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(copyright)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == email_line:
      left = (width - len(email) - 4) / 2
      right = width - len(email) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(email)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    else:
      file.write("/*")
      for j in range(width-4):
        file.write(" ")
      file.write("*/")
      file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("=")
  file.write("*/")
  file.write("\n")


def print_twin_header_info_frame(file, width, height, product, product_line, copyright, copyright_line, email, email_line, email2, email_line2, filename):
  file.write("/*")
  for j in range(width-4):
    file.write("=")
  file.write("*/")
  file.write("\n")

  for i in range(height-2):
    if i == product_line:
      left = (width - len(product) - 4) / 2
      right = width - len(product) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(product)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == copyright_line:
      left = (width - len(copyright) - 4) / 2
      right = width - len(copyright) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(copyright)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == email_line:
      left = (width - len(email) - 4) / 2
      right = width - len(email) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(email)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")

    elif i == email_line2:
      left = (width - len(email2) - 4) / 2
      right = width - len(email2) - left - 4
      file.write("/*")
      for j in range(left):
        file.write(" ")
      file.write(email2)
      for j in range(right):
        file.write(" ")
      file.write("*/")
      file.write("\n")      

    else:
      file.write("/*")
      for j in range(width-4):
        file.write(" ")
      file.write("*/")
      file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("=")
  file.write("*/")
  file.write("\n")  


def print_header_info_frame_files(file, filename, width, height, version, step, product, copyright, author, email, url):
  print_header_info_frame(file, width, height, product + " " + version, 2, copyright + " " + author, 4, url + " | " + "<" + email + ">", 5, filename)

  file_ver = filename + " / " + version

  left = 1
  right = width - len(file_ver) - left - 4

  file.write("/*")
  for j in range(left):
    file.write(" ")
  file.write(file_ver)
  for j in range(right):
    file.write(" ")
  file.write("*/")
  file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")

  
def print_twin_header_info_frame_files(file, filename, width, height, version, step, product, copyright, author, email, email2, url, url2):
  print_twin_header_info_frame(file, width, height, product + " " + version, 2, copyright + " " + author, 4, url + " | " + "<" + email + ">", 6, url2 + " | " + "<" + email2 + ">", 7, filename)

  file_ver = filename + " / " + version

  left = 1
  right = width - len(file_ver) - left - 4

  file.write("/*")
  for j in range(left):
    file.write(" ")
  file.write(file_ver)
  for j in range(right):
    file.write(" ")
  file.write("*/")
  file.write("\n")

  file.write("/*")
  for j in range(width-4):
    file.write("-")
  file.write("*/")
  file.write("\n")  


def augment_file(input_filename, output_filename, width, height, version, step, product, copyright, author, email, url, namespace, total_lines, code_lines, total_size):
  input_file = open(input_filename, "r")
  output_file = open(output_filename, "w")

  phase = 1
  php_source = 0
  php_comment = False

  for line in input_file:
    total_lines = total_lines + 1
    total_size = total_size + len(line)
    if len(line) > 1 and string.find(line, "//") < 0 and string.find(line, "/*") < 0:
      code_lines = code_lines + 1

    if string.find(line, "<?php") == 0 or string.find(line, "?>") == 0:
      php_source = php_source + 1

    if string.find(line, "<!--") == 0 or string.find(line, "-->") == 0:
      php_comment = True

    if phase == 1:
      if string.find(line, "//") != 0 and string.find(line, "/*") != 0 and (string.find(line, "<?php") != 0 or (string.find(line, "<?php") == 0 and php_source > 1)) and string.find(line, "?>") != 0 and string.find(line, "<!--") != 0 and string.find(line, "-->") != 0:
        if php_source > 0:
          output_file.write("<?php\n")
        if php_comment:
          output_file.write("<!--\n")
        print_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url)
        if php_source > 0:
          output_file.write("?>\n")
        if php_comment:
          output_file.write("-->\n")
        output_file.write(line)
        phase = 3
      elif string.find(line, "//") == 0 or (string.find(line, "/*") == 0 and len(line) <= 3):
        if php_source > 0:
          output_file.write("<?php\n")
        if php_comment:
          output_file.write("<!--\n")
        print_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url)
        output_file.write(line)
        phase = 4      
    elif phase == 2:
      if php_source > 0:
        output_file.write("<?php\n")
      if php_comment:
        output_file.write("<!--\n")
      print_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url)
      phase = 3
    elif phase == 3:
      if string.find(line, "/*--") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("-")
        output_file.write("*/")
        output_file.write("\n")
      elif string.find(line, "/*==") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("=")
        output_file.write("*/")
        output_file.write("\n")
      elif string.find(line, "namespace") == 0:
        output_file.write("namespace " + namespace + "\n")
      elif string.find(line, "} // namespace") == 0:
        output_file.write("} // namespace " + namespace + "\n")
      else:
        output_file.write(line)
    elif phase == 4:
      if string.find(line, "//") == 0:
        output_file.write(line)
      elif string.find(line, "/*--") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("-")
        output_file.write("*/")
        output_file.write("\n")        
      elif string.find(line, "/*") == 0:
        output_file.write(line)
      elif string.find(line, "*/") == 0:
        output_file.write(line)
      else:
        output_file.write(line)
        phase = 3

  input_file.close()
  output_file.close()
  return total_lines, code_lines, total_size


def twin_augment_file(input_filename, output_filename, width, height, version, step, product, copyright, author, email, url, email2, url2, namespace, total_lines, code_lines, total_size):
  input_file = open(input_filename, "r")
  output_file = open(output_filename, "w")

  phase = 1
  php_source = 0
  php_comment = False

  for line in input_file:
    total_lines = total_lines + 1
    total_size = total_size + len(line)
    if len(line) > 1 and string.find(line, "//") < 0 and string.find(line, "/*") < 0:
      code_lines = code_lines + 1

    if string.find(line, "<?php") == 0 or string.find(line, "?>") == 0:
      php_source = php_source + 1

    if string.find(line, "<!--") == 0 or string.find(line, "-->") == 0:
      php_comment = True

    if phase == 1:
      if string.find(line, "//") != 0 and string.find(line, "/*") != 0 and (string.find(line, "<?php") != 0 or (string.find(line, "<?php") == 0 and php_source > 1)) and string.find(line, "?>") != 0 and string.find(line, "<!--") != 0 and string.find(line, "-->") != 0:
        if php_source > 0:
          output_file.write("<?php\n")
        if php_comment:
          output_file.write("<!--\n")
        print_twin_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url, email2, url2)
        if php_source > 0:
          output_file.write("?>\n")
        if php_comment:
          output_file.write("-->\n")
        output_file.write(line)
        phase = 3
      elif string.find(line, "//") == 0 or (string.find(line, "/*") == 0 and len(line) <= 3):
        if php_source > 0:
          output_file.write("<?php\n")
        if php_comment:
          output_file.write("<!--\n")
        print_twin_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url, email2, url2)
        output_file.write(line)
        phase = 4      
    elif phase == 2:
      if php_source > 0:
        output_file.write("<?php\n")
      if php_comment:
        output_file.write("<!--\n")
      print_twin_header_info_frame_files(output_file, input_filename, width, height, version, step, product, copyright, author, email, url, email2, url2)
      phase = 3
    elif phase == 3:
      if string.find(line, "/*--") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("-")
        output_file.write("*/")
        output_file.write("\n")
      elif string.find(line, "/*==") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("=")
        output_file.write("*/")
        output_file.write("\n")
      elif string.find(line, "namespace") == 0:
        output_file.write("namespace " + namespace + "\n")
      elif string.find(line, "} // namespace") == 0:
        output_file.write("} // namespace " + namespace + "\n")
      else:
        output_file.write(line)
    elif phase == 4:
      if string.find(line, "//") == 0:
        output_file.write(line)
      elif string.find(line, "/*--") == 0:
        output_file.write("/*")
        for j in range(width-4):
          output_file.write("-")
        output_file.write("*/")
        output_file.write("\n")        
      elif string.find(line, "/*") == 0:
        output_file.write(line)
      elif string.find(line, "*/") == 0:
        output_file.write(line)
      else:
        output_file.write(line)
        phase = 3

  input_file.close()
  output_file.close()
  return total_lines, code_lines, total_size


def is_source(filename):
  if len(filename) >= 4 and string.find(filename, ".cpp") == len(filename) - 4:
    return True
  elif len(filename) >= 2 and (string.find(filename, ".h") == len(filename) - 2 or string.find(filename, ".hh") == len(filename) - 3):
    return True
  elif len(filename) >= 4 and string.find(filename, ".php") == len(filename) - 4 and string.find(filename, ".txt") != len(filename) - 8:
    return True
  elif len(filename) >= 4 and string.find(filename, ".css") == len(filename) - 4:
    return True
  elif len(filename) >= 5 and string.find(filename, ".html") == len(filename) - 5:
    return True
  elif len(filename) >= 3 and string.find(filename, ".js") == len(filename) - 3:
    return True
  else:
    return False


def is_exception(filename, exceptions_file):
  result = False
  exceptions_file.seek(0)

  for line in exceptions_file:
    if string.find(line, filename) == 0:
      result = True
  return result


def augment_directory(indent, directory, width, height, version, step, product, copyright, author, email, url, namespace, total_lines, code_lines, total_size, exceptions_file):
  listing = os.listdir(directory)
  for item in listing:
    if os.path.isfile(item):
      if is_source(item) and (not is_exception(item, exceptions_file)):
        print indent + "Augmenting: " + item + " ... ",
        total_lines, code_lines, total_size = augment_file(item, item + ".aug", width, height, version, step, product, copyright, author, email, url, namespace, total_lines, code_lines, total_size)
        shutil.copy(item, item + ".bak")
        shutil.move(item + ".aug", item)
        print "OK"
    elif os.path.isdir(item):
      print indent + "Subdir:" + item
      os.chdir(item)
      total_lines, code_lines, total_size = augment_directory(indent + "  ", ".", width, height, version, step, product, copyright, author, email, url, namespace, total_lines, code_lines, total_size, exceptions_file)
      os.chdir("..")
  return total_lines, code_lines, total_size


def twin_augment_directory(indent, directory, width, height, version, step, product, copyright, author, email, url, email2, url2, namespace, total_lines, code_lines, total_size, exceptions_file):
  listing = os.listdir(directory)
  for item in listing:
    if os.path.isfile(item):
      if is_source(item) and (not is_exception(item, exceptions_file)):
        print indent + "Augmenting: " + item + " ... ",
        total_lines, code_lines, total_size = twin_augment_file(item, item + ".aug", width, height, version, step, product, copyright, author, email, url, email2, url2, namespace, total_lines, code_lines, total_size)
        shutil.copy(item, item + ".bak")
        shutil.move(item + ".aug", item)
        print "OK"
    elif os.path.isdir(item):
      print indent + "Subdir:" + item
      os.chdir(item)
      total_lines, code_lines, total_size = twin_augment_directory(indent + "  ", ".", width, height, version, step, product, copyright, author, email, url, email2, url2, namespace, total_lines, code_lines, total_size, exceptions_file)
      os.chdir("..")
  return total_lines, code_lines, total_size


def print_intro():
  print "Augment 2.2 - C++/PHP Source Code Augmenter"
  print "(C) Copyright 2010-2019 Pavel Surynek"
  print "---------------------------------------------"


print_intro()

version_file = open("version", "r")
version = version_file.readline()
version_file.close()

step_file = open("step", "r")
step = step_file.readline()
step_file.close()

product_file = open("product", "r")
product = product_file.readline()
product_file.close()

copyright_file = open("copyright", "r")
copyright = copyright_file.readline()
copyright_file.close()

author_file = open("author", "r")
author = author_file.readline()
author_file.close()

email_file = open("email", "r")
email = email_file.readline()
email = email.rstrip('\n')
email2 = email_file.readline()
email2 = email2.rstrip('\n')
email_file.close()

url_file = open("url", "r")
url = url_file.readline()
url = url.rstrip('\n')
url2 = url_file.readline()
url2 = url2.rstrip('\n')
url_file.close()

namespace_file = open("namespace", "r")
namespace = namespace_file.readline()
namespace_file.close()

exceptions_file = open("exceptions", "r")
# total_lines, code_lines, total_size = augment_directory("", ".", 80, 10, version, step, product, copyright, author, email, url, namespace, 0, 0, 0, exceptions_file)
total_lines, code_lines, total_size = twin_augment_directory("", ".", 80, 11, version, step, product, copyright, author, email, email2, url, url2, namespace, 0, 0, 0, exceptions_file)
exceptions_file.close()

print
print "Total number of lines:",
print total_lines
print " Number of code lines:",
print code_lines
print "         Code density:",
density_1 = 10000 * float(code_lines) / float(total_lines)
density_2 = int(density_1)
density_3 = float(density_2) / 100
print density_3, "%"

print "           Total size:",
print total_size,
print "bytes"

total_size_kb_1 = 100 * float(total_size) / 1024
total_size_kb_2 = int(total_size_kb_1)
total_size_kb_3 = float(total_size_kb_2) / 100
print "                      ",
print total_size_kb_3,
print "Kb"

total_size_mb_1 = 100 * float(total_size) / (1024 * 1024)
total_size_mb_2 = int(total_size_mb_1)
total_size_mb_3 = float(total_size_mb_2) / 100
print "                      ",
print total_size_mb_3,
print "Mb"

print "  Average line length:",
average_1 = 100 * float(total_size) / float(total_lines)
average_2 = int(average_1)
average_3 = float(average_2) / 100
print average_3
print "."
