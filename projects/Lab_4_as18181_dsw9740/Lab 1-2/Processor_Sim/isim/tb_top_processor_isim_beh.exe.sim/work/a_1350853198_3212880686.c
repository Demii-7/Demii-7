/**********************************************************************/
/*   ____  ____                                                       */
/*  /   /\/   /                                                       */
/* /___/  \  /                                                        */
/* \   \   \/                                                       */
/*  \   \        Copyright (c) 2003-2009 Xilinx, Inc.                */
/*  /   /          All Right Reserved.                                 */
/* /---/   /\                                                         */
/* \   \  /  \                                                      */
/*  \___\/\___\                                                    */
/***********************************************************************/

/* This file is designed for use with ISim build 0x7708f090 */

#define XSI_HIDE_SYMBOL_SPEC true
#include "xsi.h"
#include <memory.h>
#ifdef __GNUC__
#include <stdlib.h>
#else
#include <malloc.h>
#define alloca _alloca
#endif
static const char *ng0 = "D:/Lab_4_as18181_dsw9740/Lab 1-2/Processor_Sim/Registers.vhd";
extern char *IEEE_P_1242562249;
extern char *IEEE_P_2592010699;

int ieee_p_1242562249_sub_1657552908_1035706684(char *, char *, char *);
unsigned char ieee_p_2592010699_sub_1744673427_503743352(char *, char *, unsigned int , unsigned int );


static void work_a_1350853198_3212880686_p_0(char *t0)
{
    char *t1;
    char *t2;
    char *t3;
    int t4;
    int t5;
    unsigned int t6;
    unsigned int t7;
    unsigned int t8;
    char *t9;
    char *t10;
    char *t11;
    char *t12;
    char *t13;
    char *t14;
    char *t15;

LAB0:    xsi_set_current_line(33, ng0);

LAB3:    t1 = (t0 + 2472U);
    t2 = *((char **)t1);
    t1 = (t0 + 1352U);
    t3 = *((char **)t1);
    t1 = (t0 + 7396U);
    t4 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t3, t1);
    t5 = (t4 - 0);
    t6 = (t5 * 1);
    xsi_vhdl_check_range_of_index(0, 7, 1, t4);
    t7 = (14U * t6);
    t8 = (0 + t7);
    t9 = (t2 + t8);
    t10 = (t0 + 4560);
    t11 = (t10 + 56U);
    t12 = *((char **)t11);
    t13 = (t12 + 56U);
    t14 = *((char **)t13);
    memcpy(t14, t9, 14U);
    xsi_driver_first_trans_fast_port(t10);

LAB2:    t15 = (t0 + 4448);
    *((int *)t15) = 1;

LAB1:    return;
LAB4:    goto LAB2;

}

static void work_a_1350853198_3212880686_p_1(char *t0)
{
    char *t1;
    char *t2;
    char *t3;
    int t4;
    int t5;
    unsigned int t6;
    unsigned int t7;
    unsigned int t8;
    char *t9;
    char *t10;
    char *t11;
    char *t12;
    char *t13;
    char *t14;
    char *t15;

LAB0:    xsi_set_current_line(34, ng0);

LAB3:    t1 = (t0 + 2472U);
    t2 = *((char **)t1);
    t1 = (t0 + 1672U);
    t3 = *((char **)t1);
    t1 = (t0 + 7428U);
    t4 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t3, t1);
    t5 = (t4 - 0);
    t6 = (t5 * 1);
    xsi_vhdl_check_range_of_index(0, 7, 1, t4);
    t7 = (14U * t6);
    t8 = (0 + t7);
    t9 = (t2 + t8);
    t10 = (t0 + 4624);
    t11 = (t10 + 56U);
    t12 = *((char **)t11);
    t13 = (t12 + 56U);
    t14 = *((char **)t13);
    memcpy(t14, t9, 14U);
    xsi_driver_first_trans_fast_port(t10);

LAB2:    t15 = (t0 + 4464);
    *((int *)t15) = 1;

LAB1:    return;
LAB4:    goto LAB2;

}

static void work_a_1350853198_3212880686_p_2(char *t0)
{
    char *t1;
    char *t2;
    unsigned char t3;
    unsigned char t4;
    char *t5;
    int t6;
    int t7;
    char *t8;
    char *t9;
    char *t10;
    int t11;
    int t12;
    unsigned int t13;
    unsigned int t14;
    unsigned int t15;
    char *t16;
    char *t17;
    char *t18;
    char *t19;
    char *t20;
    unsigned char t21;
    unsigned char t22;
    unsigned int t23;
    unsigned int t24;
    unsigned char t25;
    unsigned int t26;
    unsigned int t27;
    unsigned int t28;
    unsigned int t29;
    char *t30;
    char *t31;
    char *t32;
    char *t33;
    char *t34;

LAB0:    xsi_set_current_line(39, ng0);
    t1 = (t0 + 1192U);
    t2 = *((char **)t1);
    t3 = *((unsigned char *)t2);
    t4 = (t3 == (unsigned char)3);
    if (t4 != 0)
        goto LAB2;

LAB4:    t1 = (t0 + 992U);
    t3 = ieee_p_2592010699_sub_1744673427_503743352(IEEE_P_2592010699, t1, 0U, 0U);
    if (t3 != 0)
        goto LAB10;

LAB11:
LAB3:    t1 = (t0 + 4480);
    *((int *)t1) = 1;

LAB1:    return;
LAB2:    xsi_set_current_line(41, ng0);
    t1 = (t0 + 7670);
    *((int *)t1) = 0;
    t5 = (t0 + 7674);
    *((int *)t5) = 7;
    t6 = 0;
    t7 = 7;

LAB5:    if (t6 <= t7)
        goto LAB6;

LAB8:    goto LAB3;

LAB6:    xsi_set_current_line(42, ng0);
    t8 = xsi_get_transient_memory(14U);
    memset(t8, 0, 14U);
    t9 = t8;
    memset(t9, (unsigned char)2, 14U);
    t10 = (t0 + 7670);
    t11 = *((int *)t10);
    t12 = (t11 - 0);
    t13 = (t12 * 1);
    t14 = (14U * t13);
    t15 = (0U + t14);
    t16 = (t0 + 4688);
    t17 = (t16 + 56U);
    t18 = *((char **)t17);
    t19 = (t18 + 56U);
    t20 = *((char **)t19);
    memcpy(t20, t8, 14U);
    xsi_driver_first_trans_delta(t16, t15, 14U, 0LL);

LAB7:    t1 = (t0 + 7670);
    t6 = *((int *)t1);
    t2 = (t0 + 7674);
    t7 = *((int *)t2);
    if (t6 == t7)
        goto LAB8;

LAB9:    t11 = (t6 + 1);
    t6 = t11;
    t5 = (t0 + 7670);
    *((int *)t5) = t6;
    goto LAB5;

LAB10:    xsi_set_current_line(50, ng0);
    t2 = (t0 + 2312U);
    t5 = *((char **)t2);
    t21 = *((unsigned char *)t5);
    t22 = (t21 == (unsigned char)3);
    if (t22 == 1)
        goto LAB15;

LAB16:    t4 = (unsigned char)0;

LAB17:    if (t4 != 0)
        goto LAB12;

LAB14:
LAB13:    goto LAB3;

LAB12:    xsi_set_current_line(51, ng0);
    t18 = (t0 + 2152U);
    t19 = *((char **)t18);
    t18 = (t0 + 1992U);
    t20 = *((char **)t18);
    t18 = (t0 + 7460U);
    t11 = ieee_p_1242562249_sub_1657552908_1035706684(IEEE_P_1242562249, t20, t18);
    t12 = (t11 - 0);
    t27 = (t12 * 1);
    t28 = (14U * t27);
    t29 = (0U + t28);
    t30 = (t0 + 4688);
    t31 = (t30 + 56U);
    t32 = *((char **)t31);
    t33 = (t32 + 56U);
    t34 = *((char **)t33);
    memcpy(t34, t19, 14U);
    xsi_driver_first_trans_delta(t30, t29, 14U, 0LL);
    goto LAB13;

LAB15:    t2 = (t0 + 1992U);
    t8 = *((char **)t2);
    t2 = xsi_get_transient_memory(3U);
    memset(t2, 0, 3U);
    t9 = t2;
    if (-1 == -1)
        goto LAB18;

LAB19:    t13 = 0;

LAB20:    t6 = (t13 - 2);
    t14 = (t6 * -1);
    t15 = (1U * t14);
    t10 = (t9 + t15);
    t7 = (0 - 2);
    t23 = (t7 * -1);
    t23 = (t23 + 1);
    t24 = (1U * t23);
    memset(t10, (unsigned char)2, t24);
    t25 = 1;
    if (3U == 3U)
        goto LAB21;

LAB22:    t25 = 0;

LAB23:    t4 = (!(t25));
    goto LAB17;

LAB18:    t13 = 2;
    goto LAB20;

LAB21:    t26 = 0;

LAB24:    if (t26 < 3U)
        goto LAB25;
    else
        goto LAB23;

LAB25:    t16 = (t8 + t26);
    t17 = (t2 + t26);
    if (*((unsigned char *)t16) != *((unsigned char *)t17))
        goto LAB22;

LAB26:    t26 = (t26 + 1);
    goto LAB24;

}


extern void work_a_1350853198_3212880686_init()
{
	static char *pe[] = {(void *)work_a_1350853198_3212880686_p_0,(void *)work_a_1350853198_3212880686_p_1,(void *)work_a_1350853198_3212880686_p_2};
	xsi_register_didat("work_a_1350853198_3212880686", "isim/tb_top_processor_isim_beh.exe.sim/work/a_1350853198_3212880686.didat");
	xsi_register_executes(pe);
}
