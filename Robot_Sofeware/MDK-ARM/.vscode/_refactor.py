import os

filepath = r'e:\STM32Cube MX\RoBo_Project\Robo\MDK-ARM\User_Application\Set_And_Show\Set_And_Show.c'

with open(filepath, 'r', encoding='utf-8', newline='') as f:
    content = f.read()

# 定位旧 switch 体: 从 byte 18750 到 52034
old_body_start = 18750
old_body_end = 52034

old_body = content[old_body_start:old_body_end]
print(f"old body length: {len(old_body)} ")
print(f"old body starts with: {repr(old_body[:60])}")
print(f"old body ends with: {repr(old_body[-60:])}")

new_body = (
    '\tMenu_BlockRefresh = 0;\n'
    '\n'
    '\tif (Menu_NeedRefresh) { OLED_Clear(); Menu_NeedRefresh = 0; }\n'
    '\n'
    '\tif (Menu_Parm.Menu_Level == 0)\n'
    '\t{\n'
    '\t\tMenu_L0_OnKey();\n'
    '\t\tMenu_L0_Draw();\n'
    '\t}\n'
    '\telse if (Menu_Parm.Menu_Level == 1)\n'
    '\t{\n'
    '\t\tswitch (Menu_Parm.Menu_Level_2_Interface)\n'
    '\t\t{\n'
    '\t\t\tcase 0: Menu_L1_0_OnKey(); Menu_L1_0_Draw(); break;\n'
    '\t\t\tcase 1: Menu_L1_1_OnKey(); Menu_L1_1_Draw(); break;\n'
    '\t\t\tcase 2: Menu_L1_2_OnKey(); Menu_L1_2_Draw(); break;\n'
    '\t\t\tcase 3: Menu_L1_3_OnKey(); Menu_L1_3_Draw(); break;\n'
    '\t\t}\n'
    '\t}\n'
    '\telse if (Menu_Parm.Menu_Level == 2)\n'
    '\t{\n'
    '\t\tswitch (Menu_Parm.Menu_Level_3_Interface)\n'
    '\t\t{\n'
    '\t\t\tcase IF_L3_SpeedSet:     Menu_L2_0_OnKey();  Menu_L2_0_Draw();  break;\n'
    '\t\t\tcase IF_L3_PIDSet:       Menu_L2_1_OnKey();  Menu_L2_1_Draw();  break;\n'
    '\t\t\tcase IF_L3_GrayView:     Menu_L2_10_OnKey(); Menu_L2_10_Draw(); break;\n'
    '\t\t\tcase IF_L3_GrayCalib:    Menu_L2_11_OnKey(); Menu_L2_11_Draw(); break;\n'
    '\t\t\tcase IF_L3_ServoControl: Menu_L2_20_OnKey(); Menu_L2_20_Draw(); break;\n'
    '\t\t\tcase IF_L3_TaskLED:      Menu_L2_Task_OnKey(); Menu_L2_Task_Draw("RUN_LED_Flash", xTaskDetails_RUN_LED_Flash.usStackHighWaterMark); break;\n'
    '\t\t\tcase IF_L3_TaskKey:      Menu_L2_Task_OnKey(); Menu_L2_Task_Draw("KEY_Scan",      xTaskDetails_KEY_Scan.usStackHighWaterMark);      break;\n'
    '\t\t\tcase IF_L3_TaskShow:     Menu_L2_Task_OnKey(); Menu_L2_Task_Draw("Set_And_Show",  xTaskDetails_Set_And_Show.usStackHighWaterMark);  break;\n'
    '\t\t\tcase IF_L3_TaskUART:     Menu_L2_Task_OnKey(); Menu_L2_Task_Draw("UART_Debug",    xTaskDetails_UART_Debug.usStackHighWaterMark);    break;\n'
    '\t\t\tcase IF_L3_TaskSlow:     Menu_L2_Task_OnKey(); Menu_L2_Task_Draw("Slow_Compute",  xTaskDetails_Slow_Compute.usStackHighWaterMark);  break;\n'
    '\t\t\tcase IF_L3_TaskHigh:     Menu_L2_Task_OnKey(); Menu_L2_Task_Draw("High_Compute",  xTaskDetails_High_Compute.usStackHighWaterMark);  break;\n'
    '\t\t}\n'
    '\t}\n'
    '\n'
    '\tif (!Menu_BlockRefresh)\n'
    '\t{\n'
    '\t\tOLED_Update();\n'
    '\t\tOLED_Clear();\n'
    '\t}\n'
    '}\n'
)

new_content = content[:old_body_start] + new_body + content[old_body_end:]

with open(filepath, 'w', encoding='utf-8', newline='') as f:
    f.write(new_content)

print("File updated successfully.")
