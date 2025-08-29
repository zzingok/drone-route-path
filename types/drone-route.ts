/**
 * 倾斜摄影5航线算法 - 类型定义
 * 从 Java 版本转换而来
 * @author xhz
 */

/**
 * 经纬度点类型
 */
export interface LatLngPoint {
  latitude: number;   // 纬度
  longitude: number;  // 经度
  id?: number;        // 可选ID
}

/**
 * 倾斜摄影参数
 */
export interface ObliqueParams {
  mainDirection: number;        // 主航线方向（0-360度）
  sideOverlapRate: number;      // 旁路覆盖率（0-100%）
  forwardOverlapRate: number;   // 路线覆盖率（0-100%）
  photoWidth: number;          // 照片宽度（米）
  photoLength: number;         // 照片长度（米）
  gimbalPitch: number;         // 云台俯仰角（度，负值表示向下）
  flightHeight: number;        // 飞行高度（米）
  polygonPoints: LatLngPoint[]; // 作业区域多边形顶点
  startPoint: LatLngPoint;      // 起始点
}

/**
 * 单个方向的航线
 */
export interface RouteDirection {
  direction: number;             // 航线方向（度）
  gimbalPitch: number;          // 云台俯仰角（度）
  waypoints: LatLngPoint[];     // 航点列表
  distance: number;             // 该方向航线总距离
  description: string;          // 航线描述
}

/**
 * 倾斜摄影航线规划结果
 */
export interface ObliqueRouteResult {
  routes: RouteDirection[];       // 各方向航线列表
  totalDistance: number;          // 总距离（米）
  totalRouteCount: number;        // 实际航线数量
  isOptimized: boolean;          // 是否为优化后的航线（少于5条）
  optimizationReason: string;    // 优化原因说明
  expandedPolygon: LatLngPoint[]; // 扩展后的作业区域
  expansionDistance: number;      // 扩展距离（米）
  edgeCoverageRate: number;      // 边缘覆盖率估算
}

/**
 * 扩展区域信息
 */
export interface ExpandedAreaInfo {
  expandedPolygon: LatLngPoint[]; // 扩展后的多边形
  expansionDistance: number;      // 扩展距离（米）
  originalArea: number;           // 原始面积（平方米）
  expandedArea: number;           // 扩展后面积（平方米）
  areaIncreasePercentage: number; // 面积增加百分比
}

/**
 * 航线优化结果（内部使用）
 */
export interface RouteOptimizationResult {
  directions: number[];
  gimbalPitches: number[];
  descriptions: string[];
  isOptimized: boolean;
  reason: string;
}

/**
 * 空间网格索引类型（内部使用）
 */
export interface SpatialGridCell {
  key: string;
  points: LatLngPoint[];
}