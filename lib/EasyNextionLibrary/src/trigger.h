/*!
 * trigger.cpp - Easy library for Nextion Displays
 * Copyright (c) 2020 Athanasios Seitanis < seithagta@gmail.com >. 
 * All rights reserved under the library's licence
 */

/*! We separate this file from the EasyNextionLibrary.cpp in order to make easier the modifications for it
 * and for simplifying reasons instead of dealing with a large code file.
 
 * if you want to change the name of the predefined voids that I made,(trigger1, trigger2....etc)
 * you must change:
   1. The name from "declare the functions"  
       ---------(example: extern void trigger1(); -> extern void myFunction();)

   2. The name from "declaration of the function as weak"  
    (example: extern void trigger1() __attribute__((weak)); ---> extern void myFunction() __attribute__((weak));)

   3. the predefined name in the switch(){case} for the trigger command group at the callTriggers.cpp file
   
 * With the same way, you can add as many extern void as you like
 * NOTE: WE can have UP TO 255 extern void
 *
 * When a function has a weak attribute it will be created only when user
 * declare this function on the main code
 */


#ifndef trigger_h
#define trigger_h

// weak attribute funcion for read the custom command protocol

extern void easyNexReadCustomCommand();
extern void easyNexReadCustomCommand() __attribute__((weak));

// declare the functions for triggers
extern void trigger0();
extern void trigger1(); 
extern void trigger2();
extern void trigger3();
extern void trigger4();
extern void trigger5();
extern void trigger6();
extern void trigger7();
extern void trigger8();
extern void trigger9();
extern void trigger10();
extern void trigger11();
extern void trigger12();
extern void trigger13();
extern void trigger14();
extern void trigger15();
extern void trigger16();
extern void trigger17();
extern void trigger18();
extern void trigger19();
extern void trigger20();
extern void trigger21();
extern void trigger22();
extern void trigger23();
extern void trigger24();
extern void trigger25();
extern void trigger26();
extern void trigger27();
extern void trigger28();
extern void trigger29();
extern void trigger30();
extern void trigger31();
extern void trigger32();
extern void trigger33();
extern void trigger34();
extern void trigger35();
extern void trigger36();
extern void trigger37();
extern void trigger38();
extern void trigger39();
extern void trigger40();
extern void trigger41();
extern void trigger42();
extern void trigger43();
extern void trigger44();
extern void trigger45();
extern void trigger46();
extern void trigger47();
extern void trigger48();
extern void trigger49();
extern void trigger50();
extern void trigger51();
extern void trigger52();
extern void trigger53();
extern void trigger54();
extern void trigger55();
extern void trigger56();
extern void trigger57();
extern void trigger58();
extern void trigger59();
extern void trigger60();
extern void trigger61();
extern void trigger62();
extern void trigger63();
extern void trigger64();
extern void trigger65();
extern void trigger66();
extern void trigger67();
extern void trigger68();
extern void trigger69();
extern void trigger70();
extern void trigger71();
extern void trigger72();
extern void trigger73();
extern void trigger74();
extern void trigger75();
extern void trigger76();
extern void trigger77();
extern void trigger78();
extern void trigger79();
extern void trigger80();
extern void trigger81();
extern void trigger82();
extern void trigger83();
extern void trigger84();
extern void trigger85();
extern void trigger86();
extern void trigger87();
extern void trigger88();
extern void trigger89();
extern void trigger90();
extern void trigger91();
extern void trigger92();
extern void trigger93();
extern void trigger94();
extern void trigger95();
extern void trigger96();
extern void trigger97();
extern void trigger98();
extern void trigger99();
extern void trigger100();

// declaration of the function as weak
extern void trigger0() __attribute__((weak));
extern void trigger1() __attribute__((weak));
extern void trigger2() __attribute__((weak));
extern void trigger3() __attribute__((weak));
extern void trigger4() __attribute__((weak));
extern void trigger5() __attribute__((weak));
extern void trigger6() __attribute__((weak));
extern void trigger7() __attribute__((weak));
extern void trigger8() __attribute__((weak));
extern void trigger9() __attribute__((weak));
extern void trigger10() __attribute__((weak));
extern void trigger11() __attribute__((weak));
extern void trigger12() __attribute__((weak));
extern void trigger13() __attribute__((weak));
extern void trigger14() __attribute__((weak));
extern void trigger15() __attribute__((weak));
extern void trigger16() __attribute__((weak));
extern void trigger17() __attribute__((weak));
extern void trigger18() __attribute__((weak));
extern void trigger19() __attribute__((weak));
extern void trigger20() __attribute__((weak));
extern void trigger21() __attribute__((weak));
extern void trigger22() __attribute__((weak));
extern void trigger23() __attribute__((weak));
extern void trigger24() __attribute__((weak));
extern void trigger25() __attribute__((weak));
extern void trigger26() __attribute__((weak));
extern void trigger27() __attribute__((weak));
extern void trigger28() __attribute__((weak));
extern void trigger29() __attribute__((weak));
extern void trigger30() __attribute__((weak));
extern void trigger31() __attribute__((weak));
extern void trigger32() __attribute__((weak));
extern void trigger33() __attribute__((weak));
extern void trigger34() __attribute__((weak));
extern void trigger35() __attribute__((weak));
extern void trigger36() __attribute__((weak));
extern void trigger37() __attribute__((weak));
extern void trigger38() __attribute__((weak));
extern void trigger39() __attribute__((weak));
extern void trigger40() __attribute__((weak));
extern void trigger41() __attribute__((weak));
extern void trigger42() __attribute__((weak));
extern void trigger43() __attribute__((weak));
extern void trigger44() __attribute__((weak));
extern void trigger45() __attribute__((weak));
extern void trigger46() __attribute__((weak));
extern void trigger47() __attribute__((weak));
extern void trigger48() __attribute__((weak));
extern void trigger49() __attribute__((weak));
extern void trigger50() __attribute__((weak));
extern void trigger51() __attribute__((weak));
extern void trigger52() __attribute__((weak));
extern void trigger53() __attribute__((weak));
extern void trigger54() __attribute__((weak));
extern void trigger55() __attribute__((weak));
extern void trigger56() __attribute__((weak));
extern void trigger57() __attribute__((weak));
extern void trigger58() __attribute__((weak));
extern void trigger59() __attribute__((weak));
extern void trigger60() __attribute__((weak));
extern void trigger61() __attribute__((weak));
extern void trigger62() __attribute__((weak));
extern void trigger63() __attribute__((weak));
extern void trigger64() __attribute__((weak));
extern void trigger65() __attribute__((weak));
extern void trigger66() __attribute__((weak));
extern void trigger67() __attribute__((weak));
extern void trigger68() __attribute__((weak));
extern void trigger69() __attribute__((weak));
extern void trigger70() __attribute__((weak));
extern void trigger71() __attribute__((weak));
extern void trigger72() __attribute__((weak));
extern void trigger73() __attribute__((weak));
extern void trigger74() __attribute__((weak));
extern void trigger75() __attribute__((weak));
extern void trigger76() __attribute__((weak));
extern void trigger77() __attribute__((weak));
extern void trigger78() __attribute__((weak));
extern void trigger79() __attribute__((weak));
extern void trigger80() __attribute__((weak));
extern void trigger81() __attribute__((weak));
extern void trigger82() __attribute__((weak));
extern void trigger83() __attribute__((weak));
extern void trigger84() __attribute__((weak));
extern void trigger85() __attribute__((weak));
extern void trigger86() __attribute__((weak));
extern void trigger87() __attribute__((weak));
extern void trigger88() __attribute__((weak));
extern void trigger89() __attribute__((weak));
extern void trigger90() __attribute__((weak));
extern void trigger91() __attribute__((weak));
extern void trigger92() __attribute__((weak));
extern void trigger93() __attribute__((weak));
extern void trigger94() __attribute__((weak));
extern void trigger95() __attribute__((weak));
extern void trigger96() __attribute__((weak));
extern void trigger97() __attribute__((weak));
extern void trigger98() __attribute__((weak));
extern void trigger99() __attribute__((weak));
extern void trigger100() __attribute__((weak));

#endif