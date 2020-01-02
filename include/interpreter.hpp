#pragma once

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

class Decoder{
    private:
    public:
    float Receive(char);
}

Decoder::Receive(char buf[128]){
    float value = 0;
    char valueStr[] = "";
    float multiplier = 10.0;
    int idx = 0;
    int iddot = 0;
    char currChar = buf[idx];
    /* Advance along buffer, until "S" reached, if S reached, store remaining characters until "E" reached */
    // When "S" reached
    while (currChar != "S"){
        idx++;
        currChar = buf[idx];
    }
    // When "E" reached
    while (currChar != "E"){
        idx++;
        currChar = buf[idx];
        strcat(valueStr,currChar);
    }
    // When "." reached
    for (int i=0; i<strlen(valueStr); i++){
        if (valueStr[i] == "."){
            iddot = i;
            break;
        }
        multiplier *= 10.0;
    }

    for (int j=0; j<strlen(valueStr)-1; j++){
        int strValue = 0;
        value += multiplier * (valueStr[j] - 48);
        multiplier /= 10;
    }
    
}