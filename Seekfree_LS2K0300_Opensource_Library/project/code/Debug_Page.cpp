
#include "zf_common_headfile.h"
#include "defines.h"
#include "Key.h"


/*******************************************************************************************************************/
/*[S] 菜单样式 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// [二级界面]Debug模式界面
void Debug_Page_Menu_UI(uint8_t Page)
{
	switch(Page)
	{
		// 第一页
		case 1:
			ips200_show_string(8  , 0  , "Debug");
			ips200_show_string(0  , 16 , "==============================");
			ips200_show_string(10 , 32 , "BUZ");
		
			break;
	}
}
/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 菜单样式 [E]*/
/*******************************************************************************************************************/


/*******************************************************************************************************************/
/*[S] 交互界面 [S]--------------------------------------------------------------------------------------------------*/
/*******************************************************************************************************************/

// 相关函数提前声明
int Debug_BUZ       (void);

// [二级界面]Debug模式界面
int Debug_Page_Menu(void)
{
    // Debug模式选项 标志位
     uint8_t Debug_Page_flag = 1;

    Debug_Page_Menu_UI(1);
    ips200_show_string(0  ,32 , ">");

    while(1)
    {
        // 存储确认键被按下时Debug_Page_flag的值的临时变量，默认为无效值0
		uint8_t Debug_Page_flag_temp = 0;
		// 上/下按键是否被按下过
		uint8_t key_pressed = 0;

        /* 按键处理*/
        if (Key_Check(KEY_UP,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag --;
            if (Debug_Page_flag < 1)Debug_Page_flag = 1;
        }
        else if (Key_Check(KEY_DOWN,KEY_SINGLE))
        {
            key_pressed = 1;
            Debug_Page_flag ++;
            if (Debug_Page_flag > 1)Debug_Page_flag = 1;
        }
        else if (Key_Check(KEY_CONFIRM,KEY_SINGLE))
        {
            Debug_Page_flag_temp = Debug_Page_flag;
        }
        else if (Key_Check(KEY_BACK,KEY_SINGLE))    
        {
            // 返回上一级界面
            return 0;   
        }


        /* 模式跳转*/
        if (Debug_Page_flag_temp == 1)
        {
            ips200_clear();
            Debug_BUZ();
            
            // 从子界面返回后
            ips200_clear();
            Debug_Page_Menu_UI(1);
            ips200_show_string(0  ,32 , ">");
        }

        /* 显示更新*/
        if (key_pressed)
        {
            switch (Debug_Page_flag)
            {
                case 1:
                    Debug_Page_Menu_UI(1);
                    ips200_show_string(0  ,32 , ">");

                    break;
            }
        }
    }
}

//	####   #   #  #####
//	#   #  #   #     #  
//	####   #   #    #  
//	#   #  #   #   #   
//	####    ###   ##### 
//
// [三级界面]蜂鸣器调试
int Debug_BUZ(void)
{
    ips200_show_string(8  , 0  , "BUZ");
    ips200_show_string(0  , 16 , "==============================");
    ips200_show_string(10 , 32 , "ON ");

    uint8_t BUZ_flag = 0;

    while(1)
    {

        // 按键处理
        if (Key_Check(KEY_CONFIRM,KEY_SINGLE))
        {
            BUZ_flag = 1 - BUZ_flag;
            if (BUZ_flag)
            {
                // 开启蜂鸣器
                gpio_set_level(BEEP_NAME, 0x0);
                ips200_show_string(10 , 32 , "ON ");
            }
            else
            {
                // 关闭蜂鸣器
                gpio_set_level(BEEP_NAME, 0x1);
                ips200_show_string(10 , 32 , "OFF");
            }
        }
        else if (Key_Check(KEY_BACK,KEY_SINGLE))
        {
            // 返回上一级界面
            return 0;   
        }
    }

}
 

/*******************************************************************************************************************/
/*--------------------------------------------------------------------------------------------------[E] 交互界面 [E]*/
/*******************************************************************************************************************/