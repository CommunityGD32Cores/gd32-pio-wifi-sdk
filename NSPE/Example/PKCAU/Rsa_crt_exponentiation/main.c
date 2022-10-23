/*!
    \file    main.c
    \brief   PKCAU RSA CRT exponentiation operation

    \version 2021-10-30, V1.0.0, firmware for GD32W51x
*/

/*
    Copyright (c) 2021, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

#include "gd32w51x.h"
#include <string.h>
#include <stdio.h>
#include "gd32w515p_eval.h"

/* operand dp */
uint8_t rsa_crt_dp[]   = {0xc9, 0xd3, 0x4b, 0xe2, 0xcb, 0x8b, 0xd1, 0xca, 0x3c, 0xac, 0xb7, 0xb3, 0x82, 0xdf, 0x8f, 0x99,
                          0x41, 0xc5, 0xec, 0xeb, 0x09, 0x46, 0x31, 0x66, 0x13, 0xbe, 0x67, 0xe2, 0x05, 0x55, 0x3f, 0xc9,
                          0x90, 0xcd, 0x36, 0x42, 0xd7, 0x46, 0x21, 0x14, 0x66, 0x31, 0xac, 0xa4, 0x8f, 0x06, 0x8b, 0x5c,
                          0xa6, 0x6a, 0x95, 0xbf, 0x56, 0xe7, 0x03, 0xde, 0x0e, 0x45, 0x06, 0x5e, 0x55, 0x85, 0xe2, 0xe5};
/* operand dq */
uint8_t rsa_crt_dq[]   = {0xb4, 0x79, 0x3a, 0x0b, 0xf1, 0x37, 0xf9, 0x18, 0xb9, 0xd0, 0x12, 0xd1, 0x71, 0x22, 0x45, 0xaf,
                          0xc2, 0xf4, 0xb6, 0x64, 0x22, 0x0b, 0x7b, 0x9b, 0x5f, 0x64, 0x1f, 0x73, 0xf5, 0x1e, 0x78, 0xdf,
                          0xff, 0x46, 0x7d, 0x2f, 0x56, 0x1c, 0x3c, 0x89, 0xae, 0xcc, 0x2a, 0x82, 0x19, 0x68, 0xff, 0xf1,
                          0xb4, 0x86, 0x16, 0x93, 0x0e, 0x41, 0x6b, 0x59, 0xe5, 0x1c, 0x8b, 0x52, 0x31, 0xca, 0x63, 0x41};
/* operand qinv */
uint8_t rsa_crt_qinv[] = {0xdf, 0xaa, 0xb3, 0x84, 0xb0, 0x0e, 0x93, 0x53, 0x7f, 0x1c, 0xaf, 0x96, 0x0f, 0x39, 0x24, 0xd0,
                          0x56, 0xc0, 0x02, 0xa2, 0x29, 0x32, 0xb5, 0xc5, 0xa1, 0xac, 0xbc, 0x0d, 0x00, 0x4a, 0x43, 0x95,
                          0x56, 0x04, 0xe3, 0xc7, 0x99, 0xf8, 0x33, 0x5a, 0x3d, 0xbd, 0x05, 0x9d, 0x07, 0xcc, 0x9d, 0x1d,
                          0x13, 0xac, 0x24, 0x1d, 0xc6, 0x2d, 0xd9, 0x7a, 0xaf, 0xce, 0x17, 0xec, 0xe8, 0x7a, 0xb8, 0x78};
/* prime p */
uint8_t rsa_crt_p[]    = {0xe0, 0xbd, 0x40, 0xc5, 0x79, 0x01, 0x97, 0xeb, 0xd8, 0x7b, 0xa7, 0x75, 0x84, 0x82, 0x3d, 0x6d,
                          0xfc, 0xc9, 0x50, 0x87, 0x98, 0xf8, 0x65, 0x35, 0x98, 0xe7, 0xf9, 0xa2, 0x49, 0x88, 0x4b, 0x0d,
                          0x84, 0x59, 0xd3, 0x4b, 0x56, 0xb2, 0x92, 0xb0, 0xcf, 0xa0, 0xaa, 0xe1, 0xdc, 0x13, 0x95, 0x7b,
                          0x51, 0x0d, 0x86, 0x1a, 0xbc, 0x65, 0x02, 0xee, 0x96, 0xbf, 0x39, 0x1c, 0xde, 0x49, 0x61, 0x7d};
/* prime q */
uint8_t rsa_crt_q[]    = {0xcb, 0x19, 0x6f, 0xdf, 0x46, 0x0d, 0x83, 0xaf, 0xd4, 0xda, 0x71, 0xb3, 0xcf, 0x58, 0x48, 0x3e,
                          0x12, 0xb1, 0x87, 0xbb, 0x51, 0xfd, 0xa2, 0x2b, 0x32, 0xd5, 0x00, 0xc1, 0xa6, 0xa6, 0x94, 0x15,
                          0x22, 0xdc, 0x96, 0xae, 0x9e, 0xb0, 0xba, 0x21, 0x90, 0x70, 0xa5, 0x7d, 0x4c, 0xf1, 0x25, 0xd5,
                          0x92, 0xa1, 0x02, 0x60, 0xfa, 0x33, 0x11, 0xe5, 0xa9, 0x69, 0x4f, 0x6b, 0x70, 0xda, 0xb1, 0x31};
/* operand A */
uint8_t rsa_crt_a[]    = {0xb4, 0xbd, 0x2a, 0x41, 0x54, 0xe1, 0x23, 0x6b, 0xa2, 0xfd, 0xce, 0xca, 0x5e, 0xe9, 0x2c, 0x1b,
                          0x73, 0xdd, 0x46, 0x9a, 0x4b, 0x23, 0xa8, 0x42, 0x99, 0x43, 0xd7, 0xe9, 0x51, 0x3f, 0x15, 0xdc,
                          0x85, 0x04, 0xd5, 0x25, 0x34, 0xe3, 0x5c, 0x69, 0x1d, 0x5f, 0xb1, 0xee, 0x66, 0x08, 0x6c, 0x13,
                          0x5a, 0x8a, 0x05, 0x4b, 0x16, 0x2f, 0x32, 0x00, 0xcb, 0xaf, 0x75, 0x2e, 0xe0, 0x8c, 0xe8, 0xea,
                          0x76, 0xf6, 0xe1, 0xaa, 0x45, 0x16, 0x13, 0x80, 0x12, 0x7f, 0x2d, 0xe3, 0x87, 0x27, 0x37, 0x11,
                          0xcc, 0x79, 0x09, 0xa4, 0xb3, 0xf0, 0x46, 0x5c, 0xd9, 0xc1, 0xca, 0x8b, 0xb6, 0x66, 0x7e, 0x6c,
                          0x4f, 0x60, 0xb8, 0xa9, 0xcb, 0x3b, 0x39, 0x94, 0x21, 0x9e, 0x49, 0x63, 0x2f, 0x4a, 0xd5, 0x1a,
                          0x42, 0xba, 0xf9, 0xb9, 0x3e, 0x14, 0x35, 0x9f, 0x30, 0x28, 0x0c, 0xe7, 0x9c, 0x1f, 0xe9, 0x85};
/* expected result */
uint8_t expected_res[] = {0x22, 0xb7, 0xbc, 0x7f, 0x1c, 0xde, 0x11, 0x25, 0xb5, 0x31, 0x37, 0x51, 0x18, 0x34, 0x0d, 0x43,
                          0xdd, 0x9b, 0xf5, 0x1e, 0x78, 0x8c, 0x2a, 0x86, 0x04, 0x2d, 0x82, 0x10, 0x75, 0xd4, 0xd6, 0x9d,
                          0xca, 0xb4, 0x28, 0xee, 0x8a, 0xfb, 0xb0, 0xdf, 0x3d, 0xea, 0x61, 0xaa, 0x56, 0x96, 0x75, 0x95,
                          0xe9, 0x63, 0xcc, 0x77, 0xe3, 0xb1, 0x85, 0x63, 0x85, 0x7b, 0x9f, 0x8d, 0xca, 0xdd, 0x08, 0x88,
                          0xe3, 0x5a, 0xd9, 0x0c, 0xb5, 0x56, 0x22, 0x8c, 0x1e, 0x5a, 0x4d, 0xe5, 0x99, 0xf1, 0xb3, 0xb6,
                          0x8b, 0x3f, 0x4e, 0xbd , 0x35, 0x56, 0x4c, 0xd9, 0x8a, 0x87, 0x33, 0x74, 0x44, 0x30, 0x6e, 0xa7,
                          0x12, 0x83, 0x65, 0xab, 0x5b, 0x3e, 0x9b, 0x56, 0xc9, 0xda, 0xf0, 0x3b, 0xec, 0x3b, 0x95, 0xc4,
                          0x2f, 0xce, 0x69, 0x2e, 0x2c, 0x6f, 0x68, 0xe7, 0xe7, 0xa0, 0xdd, 0xf2, 0xa8, 0xc0, 0xc9, 0x44};

/* configure parameters of RSA CRT exponentiation operation */
void pkcau_rsa_crt_config(void);

/* operand length */
uint32_t oprd_len = sizeof(rsa_crt_a);
/*!
    \brief      main function
    \param[in]  none
    \param[out] none
    \retval     none
*/
int main(void)
{
    uint8_t rsa_crt_res[sizeof(rsa_crt_a)] = {0};
    
    /* initialize LED1 and LED2 */
    gd_eval_led_init(LED1);
    gd_eval_led_init(LED2);
    /* turn off LED1 and LED2 */
    gd_eval_led_off(LED1);
    gd_eval_led_off(LED2);
    /* enable PKCAU clock */
    rcu_periph_clock_enable(RCU_PKCAU);

    /* reset PKCAU */
    pkcau_deinit();
    /* enable PKCAU */
    pkcau_enable();
    /* wait for PKCAU busy flag to reset */ 
    while(RESET != pkcau_flag_get(PKCAU_FLAG_BUSY));
    /* configure RSA CRT exponentiation operation */
    pkcau_rsa_crt_config();
    /* wait for PKCAU operation completed */
    while(SET != pkcau_flag_get(PKCAU_INT_FLAG_END));
    /* read results from RAM address */
    pkcau_memread(0x724, rsa_crt_res, oprd_len);
    /* clear end flag */
    pkcau_flag_clear(PKCAU_INT_FLAG_END);
    /* if success, LED1 is on */
    if(memcmp(rsa_crt_res, expected_res, oprd_len)){
        gd_eval_led_on(LED2);
    }else{
        gd_eval_led_on(LED1);
    }

    while(1){
    }
}

/*!
    \brief      configure parameters of RSA CRT exponentiation operation
    \param[in]  none
    \param[out] none
    \retval     none
*/
void pkcau_rsa_crt_config(void)
{
    pkcau_crt_parameter_struct pkcau_crt_parameter;

    /* initialize the RSA CRT exponentiation parameter structure */
    pkcau_crt_struct_para_init(&pkcau_crt_parameter);

    /* initialize the input ECC curve parameters */
    pkcau_crt_parameter.oprd_a    = (uint8_t *)rsa_crt_a;
    pkcau_crt_parameter.oprd_dp   = (uint8_t *)rsa_crt_dp;
    pkcau_crt_parameter.oprd_dq   = (uint8_t *)rsa_crt_dq;
    pkcau_crt_parameter.oprd_qinv = (uint8_t *)rsa_crt_qinv;
    pkcau_crt_parameter.oprd_p    = (uint8_t *)rsa_crt_p;
    pkcau_crt_parameter.oprd_q    = (uint8_t *)rsa_crt_q;
    pkcau_crt_parameter.oprd_len  = sizeof(rsa_crt_a);
    /* execute RSA CRT exponentiation operation */
    pkcau_crt_exp_operation(&pkcau_crt_parameter);
}