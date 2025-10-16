#!/usr/bin/env python3
"""
检查PQTMPVT字段数量
"""

test_sentence = "$PQTMPVT,1,123456.789,251223,143022.123,4,1,12,18,39.123456,116.789012,123.45,45.6,0.1,0.2,-0.05,0.2236,45.67,1.2,1.5*65"

# 分割字段
fields = test_sentence.split(',')
print(f"总字段数: {len(fields)}")
print("字段列表:")
for i, field in enumerate(fields):
    print(f"  {i}: {field}")

print("\n根据PQTMPVT格式，应该有19个字段（索引0-18）:")
print("0: $PQTMPVT (句子类型)")
print("1: msg_ver (消息版本)")
print("2: tow (GPS时间周内秒数)")
print("3: date (日期)")
print("4: time (时间)")
print("5: quality (GPS定位质量)")
print("6: fix_mode (定位模式)")
print("7: num_sat_used (使用的卫星数)")
print("8: leap_s (闰秒)")
print("9: lat (纬度)")
print("10: lon (经度)")
print("11: alt (高度)")
print("12: sep (大地水准面分离)")
print("13: vel_n (北向速度)")
print("14: vel_e (东向速度)")
print("15: vel_d (天向速度)")
print("16: spd (地面速度)")
print("17: heading (航向角)")
print("18: hdop (水平精度因子)")
print("19: pdop*checksum (位置精度因子+校验和)")

print(f"\n实际字段数: {len(fields)}")
print(f"期望字段数: 20 (包括校验和)")
print(f"数据字段数: 19 (不包括校验和)")
