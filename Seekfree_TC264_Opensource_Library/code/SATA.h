/*
 * SATA.h
 *
 *  Created on: 2026年10月19日
 *      Author: Freed
 */

#ifndef CODE_SATA_H_
#define CODE_SATA_H_
#include "zf_common_headfile.h"

void SATA_MAIN(void);                                   // 初始化：摄像头 + TFT180
void SATA_Loop(void);                                   // 放进主循环：每来一帧处理并显示一次

#endif /* CODE_SATA_H_ */
