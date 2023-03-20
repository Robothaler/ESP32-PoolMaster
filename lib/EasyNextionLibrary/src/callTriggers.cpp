/*!
 * callTriggers.cpp - Easy library for Nextion Displays
 * Copyright (c) 2020 Athanasios Seitanis < seithagta@gmail.com >. 
 * All rights reserved under the library's licence
 */

/*! We made a separate file < callTriggers.cpp > for the function callTriggerFunction(), 
 *  as it would be too long to fit in the EasyNextionLibrary.cpp, because there are
 *  50 predefined cases for the triggers and it is easier for someone to edit them
 */

#ifndef EasyNextionLibrary_h
#include "EasyNextionLibrary.h"
#endif                        
                                
#ifndef trigger_h
#include "trigger.h"
#endif

void EasyNex::callTriggerFunction(){
  
 uint8_t _tempRead = _serial->read();  // We read the next byte, which, according to our protocol, is the < Trigger ID >
                                // From Nextion we send: < printh 23 02 54 xx >
                                // (where xx is the trigger id in HEX, 01 for 1, 02 for 2, ... 0A for 10 etc)
  switch(_tempRead){
    
    case 0x00:
      trigger0();
      break;    
    
    case 0x01:
      trigger1();
      break;

    case 0x02:
      trigger2();
      break;
           
    case 0x03:
      trigger3();
      break;
                 
    case 0x04:
      trigger4();
      break;    
                 
    case 0x05:
      trigger5();
      break;   
    
    case 0x06:
      trigger6();
      break;

    case 0x07:
      trigger7();
      break;
           
    case 0x08:
      trigger8();
      break;
                 
    case 0x09:
      trigger9();
      break;    
                 
    case 0x0A:
      trigger10();
      break;
    
    case 0x0B:
      trigger11();
      break;

    case 0x0C:
      trigger12();
      break;
           
    case 0x0D:
      trigger13();
      break;
                 
    case 0x0E:
      trigger14();
      break;    
                 
    case 0x0F:
      trigger15();
      break;   
    
    case 0x10:
      trigger16();
      break;

    case 0x11:
      trigger17();
      break;
           
    case 0x12:
      trigger18();
      break;
                 
    case 0x13:
      trigger19();
      break;    
                 
    case 0x14:
      trigger20();
      break; 

    case 0x15:
      trigger21();
      break;

    case 0x16:
      trigger22();
      break;
           
    case 0x17:
      trigger23();
      break;
                 
    case 0x18:
      trigger24();
      break;    
                 
    case 0x19:
      trigger25();
      break;   
    
    case 0x1A:
      trigger26();
      break;

    case 0x1B:
      trigger27();
      break;
           
    case 0x1C:
      trigger28();
      break;
                 
    case 0x1D:
      trigger29();
      break;    
                 
    case 0x1E:
      trigger30();
      break;
    
    case 0x1F:
      trigger31();
      break;

    case 0x20:
      trigger32();
      break;
           
    case 0x21:
      trigger33();
      break;
                 
    case 0x22:
      trigger34();
      break;    
                 
    case 0x23:
      trigger35();
      break;   
    
    case 0x24:
      trigger36();
      break;

    case 0x25:
      trigger37();
      break;
           
    case 0x26:
      trigger38();
      break;
                 
    case 0x27:
      trigger39();
      break;    
                 
    case 0x28:
      trigger40();
      break; 
    
    case 0x29:
      trigger41();
      break;

    case 0x2A:
      trigger42();
      break;
           
    case 0x2B:
      trigger43();
      break;
                 
    case 0x2C:
      trigger44();
      break;    
                 
    case 0x2D:
      trigger45();
      break;   
    
    case 0x2E:
      trigger46();
      break;

    case 0x2F:
      trigger47();
      break;
           
    case 0x30:
      trigger48();
      break;
                 
    case 0x31:
      trigger49();
      break;    
                 
    case 0x32:
      trigger50();
      break;

    case 0x33:
      trigger51();
      break;

    case 0x34:
      trigger52();
      break;

    case 0x35:
      trigger53();
      break;

    case 0x36:
      trigger54();
      break;

    case 0x37:
      trigger55();
      break;

    case 0x38:
      trigger56();
      break;

    case 0x39:
      trigger57();
      break;

    case 0x3A:
      trigger58();
      break;

    case 0x3B:
      trigger59();
      break;

    case 0x3C:
      trigger60();
      break;

    case 0x3D:
      trigger61();
      break;

    case 0x3E:
      trigger62();
      break;

    case 0x3F:
      trigger63();
      break;

    case 0x40:
      trigger64();
      break;

    case 0x41:
      trigger65();
      break;

    case 0x42:
      trigger66();
      break;

    case 0x43:
      trigger67();
      break;

    case 0x44:
      trigger68();
      break;

    case 0x45:
      trigger69();
      break;

    case 0x46:
      trigger70();
      break;

    case 0x47:
      trigger71();
      break;

    case 0x48:
      trigger72();
      break;

    case 0x49:
      trigger73();
      break;

    case 0x4A:
      trigger74();
      break;

    case 0x4B:
      trigger75();
      break;

    case 0x4C:
      trigger76();
      break;

    case 0x4D:
      trigger77();
      break;

    case 0x4E:
      trigger78();
      break;

    case 0x4F:
      trigger79();
      break;

    case 0x50:
      trigger80();
      break;

    case 0x51:
      trigger81();
      break;

    case 0x52:
      trigger82();
      break;

    case 0x53:
      trigger83();
      break;

    case 0x54:
      trigger84();
      break;

    case 0x55:
      trigger85();
      break;

    case 0x56:
      trigger86();
      break;

    case 0x57:
      trigger87();
      break;

    case 0x58:
      trigger88();
      break;

    case 0x59:
      trigger89();
      break;

    case 0x5A:
      trigger90();
      break;

    case 0x5B:
      trigger91();
      break;

    case 0x5C:
      trigger92();
      break;

    case 0x5D:
      trigger93();
      break;

    case 0x5E:
      trigger94();
      break;

    case 0x5F:
      trigger95();
      break;

    case 0x60:
      trigger96();
      break;

    case 0x61:
      trigger97();
      break;

    case 0x62:
      trigger98();
      break;

    case 0x63:
      trigger99();
      break;

    case 0x64:
      trigger100();
      break;
      }
}