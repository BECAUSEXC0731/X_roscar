#ifndef SELF_CHECK_H
#define SELF_CHECK_H

#include <Arduino.h>

/**
 * 执行上电自检，返回 true 表示全部正常
 */
bool runSelfCheck();

/**
 * 控制蜂鸣器鸣叫指定时长（毫秒）
 */
void beepBuzzer(uint16_t durationMs);

#endif
