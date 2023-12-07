// Lab 5
// Edgar Hernandez
// Modified code originally from Jason Losh

#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "clock.h"
#include "uart0.h"
#include "tm4c123gh6pm.h"
#include "uartparse.h"

#define DEBUG

void getsUart0(USER_DATA *data)          //save string to buffer
{
    int count = 0;
    while(count<MAX_CHARS)
    {
        char c = getcUart0();
        if(c==8 || c==127 && count>0)
        {
            count--;
        }
        else if(c==13)
        {
            data->buffer[count] = '\0';
            count = MAX_CHARS;
        }
        else if(c>=32)
        {
            data->buffer[count] = c;
            count++;
        }
    }
    data->buffer[count] = '\0';
}
char charType(char c)           //return type of character
{
    if(c>47 && c<58 || c==46)
    {
        return 'n';
    }
    else if(c>64 && c<99 || c>96 && c<123)
    {
        return 'a';
    }
    else return 'd';
}
void parseFields(USER_DATA *data)    //set field types and positions and set delims to \0
{
    int count = 1;
    int i = 1;
    if(data->buffer[0] != '\0')
    {
        data->fieldPosition[0] = 0;
        data->fieldType[0] = charType(data->buffer[0]);
        while(data->buffer[i] != '\0')
        {
            if((charType(data->buffer[i]) != charType(data->buffer[i-1])) && charType(data->buffer[i]) != 'd')
            {
                data->fieldPosition[count] = i;
                data->fieldType[count] = charType(data->buffer[i]);
                count++;
            }
            i++;
        }
        data->fieldCount = count;
    }
    i = 0;
    while(data->buffer[i] != '\0')
    {
        if(charType(data->buffer[i]) == 'd')
        {
            data->buffer[i] = '\0';
        }
        i++;
    }
}

char* getFieldString(USER_DATA* data, uint8_t fieldNumber)   //return pointer to field
{
    if(data->fieldCount >= fieldNumber)
        {
            return &(data->buffer[data->fieldPosition[fieldNumber]]);
        }
    else return NULL;
}

int32_t getFieldInteger(USER_DATA* data, uint8_t fieldNumber)    //return field integer
{
    if(data->fieldCount >= fieldNumber && data->fieldType[fieldNumber] == 'n')
    {
        return atoi(&(data->buffer[data->fieldPosition[fieldNumber]]));
    }
    else return 0;
}

float getFieldFloat(USER_DATA* data, uint8_t fieldNumber)
{
    if(data->fieldCount >= fieldNumber && data->fieldType[fieldNumber] == 'n')
    {
        return atof(&(data->buffer[data->fieldPosition[fieldNumber]]));
    }
    else return 0;
}

bool isCommand(USER_DATA* data, const char strCommand[],uint8_t minArguments) //check if first field matches commands
{
    int i;
    bool comm = true;
    int commLen = 0;

    while(strCommand[commLen] != '\0')
    {
        commLen++;
    }

    for(i=0; i<commLen+1; i++)
    {
        if(data->buffer[i] != strCommand[i])
        {
            comm = false;
        }
    }
    if(data->fieldCount - 1 < minArguments)
    {
        comm = false;
    }
    return comm;
}

