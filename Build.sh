#!/bin/bash

#/************************/
#/* Compile application. */
#/************************/
gpasm -w 1 -c PicNeoLED.asm

if [ $? -eq 0 ]
then
#/*********************/
#/* Link application. */
#/*********************/
   gplink -o PicNeoLED.hex PicNeoLED.o

   if [ $? -eq 0 ]
   then
#/******************************/
#/* Simulate test application. */
#/******************************/
      gpsim -s PicNeoLED.cod
   fi
fi

