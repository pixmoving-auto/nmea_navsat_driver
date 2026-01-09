import pyproj
import math
import numpy as np

DEG_TO_RAD = 0.01745329252
EARTH_MAJOR = 6378137.0
EARTH_MINOR = 6356752.31424518

ECEF = pyproj.Proj(proj='geocent', ellps='WGS84', datum='WGS84')
LLA = pyproj.Proj(proj='latlong', ellps='WGS84', datum='WGS84')

def lla2ecef(lon, lat, alt):
  x, y, z = pyproj.transform(LLA, ECEF, lon, lat, alt)
  return x, y, z

def lla2ecef_simple(lon, lat, alt):
  lat = math.radians(lat)
  lon = math.radians(lon)
  earth_r = math.pow(EARTH_MAJOR, 2) / math.sqrt(pow(EARTH_MAJOR * math.cos(lat), 2) + math.pow(EARTH_MINOR * math.sin(lat), 2))
  x = (earth_r + alt) * math.cos(lat) * math.cos(lon)
  y = (earth_r + alt) * math.cos(lat) * math.sin(lon)
  z = (math.pow(EARTH_MINOR / EARTH_MAJOR, 2) * earth_r + alt) * math.sin(lat)
  return x, y, z

def lla_cov_to_ecef(cov_lla_diag, lon, lat, alt):
    """
    将LLA对角协方差转换为ECEF对角协方差
    
    Args:
        cov_lla_diag: [var_lon_deg2, var_lat_deg2, var_alt_m2]
        lon, lat, alt: 参考点坐标 (deg, deg, m)
        
    Returns:
        cov_ecef_diag: [var_x_m2, var_y_m2, var_z_m2]
    """
    # 1. 构建LLA协方差矩阵（转换为弧度方差）
    var_lon_rad2 = cov_lla_diag[0] * (DEG_TO_RAD**2)
    var_lat_rad2 = cov_lla_diag[1] * (DEG_TO_RAD**2)
    var_alt_m2 = cov_lla_diag[2]
    
    cov_lla = np.diag([var_lon_rad2, var_lat_rad2, var_alt_m2])
    
    # 2. 使用数值微分计算雅可比矩阵
    # 参考点
    x0, y0, z0 = pyproj.transform(LLA, ECEF, lon, lat, alt)
    
    # 微分量
    eps_deg = 1e-8  # 角度微分量（度）
    eps_m = 1e-4    # 高程微分量（米）
    
    # 计算偏导数
    # d/dlon
    x_lon, y_lon, z_lon = pyproj.transform(LLA, ECEF, lon+eps_deg, lat, alt)
    dx_dlon = (x_lon - x0) / eps_deg
    dy_dlon = (y_lon - y0) / eps_deg
    dz_dlon = (z_lon - z0) / eps_deg
    
    # d/dlat
    x_lat, y_lat, z_lat = pyproj.transform(LLA, ECEF, lon, lat+eps_deg, alt)
    dx_dlat = (x_lat - x0) / eps_deg
    dy_dlat = (y_lat - y0) / eps_deg
    dz_dlat = (z_lat - z0) / eps_deg
    
    # d/dalt
    x_alt, y_alt, z_alt = pyproj.transform(LLA, ECEF, lon, lat, alt+eps_m)
    dx_dalt = (x_alt - x0) / eps_m
    dy_dalt = (y_alt - y0) / eps_m
    dz_dalt = (z_alt - z0) / eps_m
    
    # 雅可比矩阵 [3×3]
    J = np.array([
        [dx_dlon, dx_dlat, dx_dalt],
        [dy_dlon, dy_dlat, dy_dalt],
        [dz_dlon, dz_dlat, dz_dalt]
    ])
    
    # 3. 协方差传播：Cov_xyz = J × Cov_lla × J^T
    cov_xyz = J @ cov_lla @ J.T
    
    # 返回对角线元素
    return np.diag(cov_xyz)

if __name__ == '__main__':

  import time
  start = time.time()
  for i in range(1000):
    x, y, z = lla2ecef(106.2, 26.0, 1234.0)
  end = time.time()
  print(1.0/((end-start)/1000))
  print(lla2ecef(106.2, 26.4, 1231.5))
  start = time.time()
  for i in range(1000):
    x, y, z = lla2ecef_simple(106.2, 26.4, 1231.5)
  end = time.time()
  print(1.0/((end-start)/1000))
  print(lla2ecef_simple(106.2, 26.4, 1231.5))
  