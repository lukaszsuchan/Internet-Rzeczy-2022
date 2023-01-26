# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/lsuchan/esp/esp-idf/components/bootloader/subproject"
  "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader"
  "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader-prefix"
  "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader-prefix/tmp"
  "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader-prefix/src/bootloader-stamp"
  "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader-prefix/src"
  "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/lsuchan/Dokumenty/studia/iot/Internet-Rzeczy-2022/external-thermometer/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
