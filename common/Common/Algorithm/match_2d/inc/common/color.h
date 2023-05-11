#ifndef COLOR_H
#define COLOR_H

#include <stdio.h>
#define MAPPER_NONE "\e[0m"        //清除颜色，之后的打印为正常输出，之前的不受影响
#define MAPPER_L_RED "\e[1;31m"    //鲜红
#define MAPPER_YELLOW "\e[1;33m"   //鲜黄
#define MAPPER_BLUE "\e[0;34m"     //深蓝，暗蓝
//
#define MAPPER_INFO(s) std::cout << MAPPER_BLUE << s << MAPPER_NONE << std::endl;
#define MAPPER_WARN(s) std::cout << MAPPER_YELLOW << s << MAPPER_NONE << std::endl;
#define MAPPER_ERRO(s) std::cout << MAPPER_L_RED << s << MAPPER_NONE << std::endl;

#endif // COLOR_H
