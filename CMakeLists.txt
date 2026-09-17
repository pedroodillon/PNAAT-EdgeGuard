cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(p101_bno085_poc)

target_add_binary_data(
    p101_bno085_poc.elf
    "main/certs/AmazonRootCA1.pem"
    TEXT
)

target_add_binary_data(
    p101_bno085_poc.elf
    "main/certs/device-cert.pem.crt"
    TEXT
)

target_add_binary_data(
    p101_bno085_poc.elf
    "main/certs/device-private.pem.key"
    TEXT
)