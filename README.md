# ST_Examples

Here are the projects that I did with STM32-L476RGxx-Nucleo 64 board. They are not the bests but they are all working :D 

01. Toggle led through GPIO A4 pin of the ST (External Led). High level coding with HAL libraries.
02. Toggle 3 Leds simultaneously exploiting RTOS_CMSIS_V2. Pins used are all from bank A GPIO which are PIN_1, PIN_4 and PIN_5. Two tasks are done by the MxCube and I added another task to practice. High level coding with HAL libraries.
03. Read Analog value of a touch sensor via ADC and using two buffer ping-ponging feature of DMA to process analog inputs. 
