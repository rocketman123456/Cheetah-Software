cmake_minimum_required(VERSION 3.0)

project(bcm2835 LANGUAGES C)

option(BUILD_SHARED_LIBS "Create shared libraries" ON)

add_library(bcm2835 bcm2835.c)
