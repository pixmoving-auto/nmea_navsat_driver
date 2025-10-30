#!/usr/bin/env python3
"""
测试DRPVA数据解析
用法: python3 test_drpva.py
"""
import re

def safe_float(field):
    try:
        return float(field)
    except ValueError:
        return float('NaN')

def safe_float_except_star(field):
    try:
        return float(field.split('*')[0])
    except ValueError:
        return float('NaN')

def check_nmea_checksum(nmea_sentence):
    """校验NMEA句子的校验和"""
    if not re.match(r'.*\*[0-9A-Fa-f]{2}$', nmea_sentence):
        return False
    
    # 提取数据部分和校验和
    parts = nmea_sentence.split('*')
    if len(parts) != 2:
        return False
    
    data = parts[0]
    checksum_str = parts[1]
    
    # 计算校验和 (去掉$符号)
    if data.startswith('$'):
        data = data[1:]
    
    checksum = 0
    for char in data:
        checksum ^= ord(char)
    
    # 比较
    calculated = format(checksum, '02X')
    return calculated.upper() == checksum_str.upper()

def parse_drpva(nmea_sentence):
    """解析DRPVA NMEA句子"""
    fields = nmea_sentence.split(',')
    
    if len(fields) < 16:
        return None
    
    # 提取最后一个字段的校验和
    last_field = fields[15].split('*')
    
    parsed = {
        'sentence_type': fields[0],
        'msg_ver': int(fields[1]),
        'tow': float(fields[2]),
        'utc_time': float(fields[3]),
        'quality': int(fields[4]),
        'latitude': float(fields[5]),
        'longitude': float(fields[6]),
        'altitude': float(fields[7]),
        'vel_horiz': float(fields[8]),
        'lat_std_dev': float(fields[9]),
        'lon_std_dev': float(fields[10]),
        'alt_std_dev': float(fields[11]),
        'vel_std_dev': float(fields[12]),
        'heading': float(fields[13]),
        'pitch': float(fields[14]),
        'roll': float(last_field[0]),
        'checksum': last_field[1] if len(last_field) > 1 else 'N/A'
    }
    
    return parsed

def quality_to_status(quality):
    """将质量等级转换为状态描述"""
    status_map = {
        0: "无定位 (NO_FIX)",
        1: "GPS定位 (FIX)",
        2: "差分GPS (DGPS/SBAS_FIX)",
        4: "RTK固定解 (RTK Fixed/GBAS_FIX)",
        5: "RTK浮点解 (RTK Float/GBAS_FIX)"
    }
    return status_map.get(quality, f"未知质量等级 ({quality})")

# 测试数据
test_sentence = "$PQTMDRPVA,1,1534581,062343.400,2,26.74837099,106.66894064,1270.464,0.000,0.557,0.268,0.637,0.618,118.013,11.383,248.324*5B"

print("=" * 80)
print("DR PVA (PQTMDRPVA) 数据解析测试")
print("=" * 80)
print(f"\n原始NMEA句子:\n{test_sentence}")
print(f"\n句子长度: {len(test_sentence)} 字符")

# 检查校验和
checksum_valid = check_nmea_checksum(test_sentence)
print(f"\n校验和检查: {'✓ 通过' if checksum_valid else '✗ 失败'}")

# 解析数据
parsed = parse_drpva(test_sentence)

if parsed:
    print("\n" + "=" * 80)
    print("解析结果:")
    print("=" * 80)
    
    print(f"\n【基本信息】")
    print(f"  句子类型:        {parsed['sentence_type']}")
    print(f"  消息版本:        {parsed['msg_ver']}")
    print(f"  GPS周内秒数:     {parsed['tow']}")
    print(f"  UTC时间:         {parsed['utc_time']} (HHMMSS.SSS格式)")
    
    print(f"\n【定位质量】")
    print(f"  质量等级:        {parsed['quality']} - {quality_to_status(parsed['quality'])}")
    
    print(f"\n【位置信息】")
    print(f"  纬度:           {parsed['latitude']:.8f}°")
    print(f"  经度:           {parsed['longitude']:.8f}°")
    print(f"  高度:           {parsed['altitude']:.3f} 米")
    
    print(f"\n【速度信息】")
    print(f"  水平速度:        {parsed['vel_horiz']:.3f} m/s")
    
    print(f"\n【精度信息 (标准差)】")
    print(f"  纬度标准差:      {parsed['lat_std_dev']:.3f} 米")
    print(f"  经度标准差:      {parsed['lon_std_dev']:.3f} 米")
    print(f"  高度标准差:      {parsed['alt_std_dev']:.3f} 米")
    print(f"  速度标准差:      {parsed['vel_std_dev']:.3f} m/s")
    
    print(f"\n【姿态信息】")
    print(f"  航向角:         {parsed['heading']:.3f}°")
    print(f"  俯仰角:         {parsed['pitch']:.3f}°")
    print(f"  横滚角:         {parsed['roll']:.3f}°")
    
    print(f"\n【协方差矩阵值】")
    print(f"  position_covariance[0] (经度): {parsed['lon_std_dev']**2:.6f}")
    print(f"  position_covariance[4] (纬度): {parsed['lat_std_dev']**2:.6f}")
    print(f"  position_covariance[8] (高度): {parsed['alt_std_dev']**2:.6f}")
    
    print(f"\n【校验和】")
    print(f"  校验和:         {parsed['checksum']}")
    
else:
    print("\n✗ 解析失败!")

print("\n" + "=" * 80)
print("测试完成!")
print("=" * 80)
print("\n提示:")
print("- parser.py 已添加PQTMDRPVA解析配置")
print("- driver.py 已将GGA处理替换为DRPVA处理")
print("- 可以通过ROS2节点订阅 /fix 话题来查看实际发布的数据")
print("=" * 80)

