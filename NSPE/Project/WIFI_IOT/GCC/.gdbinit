target remote :2331
set mem inaccessible-by-default off
monitor reset
load
break Reset_Handler
break main
continue