/**
 * 倾斜摄影5航线算法 - 核心工具函数
 * 从 Java 版本转换而来
 * @author xhz
 */

import type { 
  LatLngPoint, 
  ObliqueParams, 
  RouteOptimizationResult,
  SpatialGridCell 
} from '~/types/drone-route'

// 常量定义
export const EARTH_RADIUS = 6371000 // 地球半径（米）
export const STANDARD_5_DIRECTIONS = [0, 72, 144, 216, 288] // 标准5方向（度）
export const MIN_EFFECTIVE_PITCH = -15.0 // 最小有效俯仰角
export const MAX_EFFECTIVE_PITCH = -60.0 // 最大有效俯仰角

// 性能优化：缓存
const DISTANCE_CACHE = new Map<string, number>()
const COVERAGE_RADIUS_CACHE = new Map<string, number>()

// 预计算三角函数表
const COS_TABLE: number[] = []
const SIN_TABLE: number[] = []

// 初始化三角函数表
for (let i = 0; i <= 360; i++) {
  const rad = (i * Math.PI) / 180
  COS_TABLE[i] = Math.cos(rad)
  SIN_TABLE[i] = Math.sin(rad)
}

/**
 * 空间网格索引类
 */
export class SpatialGrid {
  private cellSize: number
  private grid: Map<string, LatLngPoint[]>

  constructor(cellSize: number) {
    this.cellSize = cellSize
    this.grid = new Map()
  }

  addPoint(point: LatLngPoint): void {
    const key = this.getCellKey(point)
    if (!this.grid.has(key)) {
      this.grid.set(key, [])
    }
    this.grid.get(key)!.push(point)
  }

  private getCellKey(point: LatLngPoint): string {
    const x = Math.floor(point.longitude / this.cellSize)
    const y = Math.floor(point.latitude / this.cellSize)
    return `${x},${y}`
  }
}

/**
 * 快速三角函数计算
 */
export function fastCos(degrees: number): number {
  let index = Math.round(degrees) % 360
  if (index < 0) {
    index += 360
  }
  return COS_TABLE[index] || 0
}

export function fastSin(degrees: number): number {
  let index = Math.round(degrees) % 360
  if (index < 0) {
    index += 360
  }
  return SIN_TABLE[index] || 0
}

/**
 * 计算两点间距离（Haversine公式）
 */
export function calculateDistance(point1: LatLngPoint, point2: LatLngPoint): number {
  const lat1Rad = (point1.latitude * Math.PI) / 180
  const lat2Rad = (point2.latitude * Math.PI) / 180
  const deltaLatRad = ((point2.latitude - point1.latitude) * Math.PI) / 180
  const deltaLngRad = ((point2.longitude - point1.longitude) * Math.PI) / 180

  const a = Math.sin(deltaLatRad / 2) * Math.sin(deltaLatRad / 2) +
            Math.cos(lat1Rad) * Math.cos(lat2Rad) *
            Math.sin(deltaLngRad / 2) * Math.sin(deltaLngRad / 2)
  const c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a))

  return EARTH_RADIUS * c
}

/**
 * 缓存的距离计算
 */
export function calculateDistanceCached(point1: LatLngPoint, point2: LatLngPoint): number {
  const key = `${point1.latitude.toFixed(6)},${point1.longitude.toFixed(6)}-${point2.latitude.toFixed(6)},${point2.longitude.toFixed(6)}`
  
  if (!DISTANCE_CACHE.has(key)) {
    DISTANCE_CACHE.set(key, calculateDistance(point1, point2))
  }
  
  return DISTANCE_CACHE.get(key)!
}

/**
 * 快速距离计算（近距离使用平面近似）
 */
export function calculateDistanceFast(point1: LatLngPoint, point2: LatLngPoint): number {
  const deltaLat = point2.latitude - point1.latitude
  const deltaLng = point2.longitude - point1.longitude
  
  // 对于小距离，使用平面近似计算（误差<1%）
  if (Math.abs(deltaLat) < 0.01 && Math.abs(deltaLng) < 0.01) {
    const avgLat = (point1.latitude + point2.latitude) / 2
    const latDistance = deltaLat * 111000
    const lngDistance = deltaLng * 111000 * Math.cos((avgLat * Math.PI) / 180)
    return Math.sqrt(latDistance * latDistance + lngDistance * lngDistance)
  }
  
  // 大距离仍使用精确的Haversine公式
  return calculateDistance(point1, point2)
}

/**
 * 边界框检查优化
 */
export function isPointInBoundingBox(point: LatLngPoint, polygon: LatLngPoint[]): boolean {
  let minLat = Number.MAX_VALUE
  let maxLat = Number.MIN_VALUE
  let minLng = Number.MAX_VALUE
  let maxLng = Number.MIN_VALUE
  
  for (const p of polygon) {
    minLat = Math.min(minLat, p.latitude)
    maxLat = Math.max(maxLat, p.latitude)
    minLng = Math.min(minLng, p.longitude)
    maxLng = Math.max(maxLng, p.longitude)
  }
  
  return point.latitude >= minLat && point.latitude <= maxLat &&
         point.longitude >= minLng && point.longitude <= maxLng
}

/**
 * 边界框附近检查
 */
export function isPointNearBoundingBox(point: LatLngPoint, polygon: LatLngPoint[], threshold: number): boolean {
  let minLat = Number.MAX_VALUE
  let maxLat = Number.MIN_VALUE
  let minLng = Number.MAX_VALUE
  let maxLng = Number.MIN_VALUE
  
  for (const p of polygon) {
    minLat = Math.min(minLat, p.latitude)
    maxLat = Math.max(maxLat, p.latitude)
    minLng = Math.min(minLng, p.longitude)
    maxLng = Math.max(maxLng, p.longitude)
  }
  
  // 将阈值转换为经纬度
  const latThreshold = threshold / 111000.0
  const avgLat = (minLat + maxLat) / 2
  const lngThreshold = threshold / (111000.0 * Math.cos((avgLat * Math.PI) / 180))
  
  return point.latitude >= (minLat - latThreshold) && point.latitude <= (maxLat + latThreshold) &&
         point.longitude >= (minLng - lngThreshold) && point.longitude <= (maxLng + lngThreshold)
}

/**
 * 检查点是否在多边形内部（射线法）
 */
export function isPointInPolygon(point: LatLngPoint, polygon: LatLngPoint[]): boolean {
  // 先进行边界框检查，快速排除明显不在多边形内的点
  if (!isPointInBoundingBox(point, polygon)) {
    return false
  }
  
  let intersections = 0
  const n = polygon.length
  
  for (let i = 0; i < n; i++) {
    const p1 = polygon[i]
    const p2 = polygon[(i + 1) % n]
    
    if (((p1.latitude > point.latitude) !== (p2.latitude > point.latitude)) &&
        (point.longitude < (p2.longitude - p1.longitude) * (point.latitude - p1.latitude) / (p2.latitude - p1.latitude) + p1.longitude)) {
      intersections++
    }
  }
  
  return (intersections % 2) === 1
}

/**
 * 计算点到线段的距离
 */
export function distanceToLineSegment(point: LatLngPoint, lineStart: LatLngPoint, lineEnd: LatLngPoint): number {
  const A = point.latitude - lineStart.latitude
  const B = point.longitude - lineStart.longitude
  const C = lineEnd.latitude - lineStart.latitude
  const D = lineEnd.longitude - lineStart.longitude
  
  const dot = A * C + B * D
  const lenSq = C * C + D * D
  
  if (lenSq === 0) {
    return calculateDistanceFast(point, lineStart)
  }
  
  const param = dot / lenSq
  
  if (param < 0) {
    return calculateDistanceFast(point, lineStart)
  } else if (param > 1) {
    return calculateDistanceFast(point, lineEnd)
  } else {
    const projection: LatLngPoint = {
      latitude: lineStart.latitude + param * C,
      longitude: lineStart.longitude + param * D
    }
    
    return calculateDistanceFast(point, projection)
  }
}

/**
 * 检查点是否接近多边形
 */
export function isPointNearPolygon(point: LatLngPoint, polygon: LatLngPoint[], threshold: number): boolean {
  // 检查点是否在多边形内
  if (isPointInPolygon(point, polygon)) {
    return true
  }
  
  // 检查点到多边形边界的最短距离
  for (let i = 0; i < polygon.length; i++) {
    const p1 = polygon[i]
    const p2 = polygon[(i + 1) % polygon.length]
    
    if (distanceToLineSegment(point, p1, p2) <= threshold) {
      return true
    }
  }
  
  return false
}

/**
 * 计算航线总距离
 */
export function calculateRouteDistance(waypoints: LatLngPoint[]): number {
  if (waypoints.length < 2) {
    return 0
  }
  
  let totalDistance = 0
  for (let i = 1; i < waypoints.length; i++) {
    totalDistance += calculateDistanceCached(waypoints[i - 1], waypoints[i])
  }
  
  return totalDistance
}

/**
 * 计算多边形面积（平方米）
 */
export function calculatePolygonArea(polygon: LatLngPoint[]): number {
  if (polygon.length < 3) {
    return 0
  }
  
  let area = 0
  const n = polygon.length
  
  for (let i = 0; i < n; i++) {
    const p1 = polygon[i]
    const p2 = polygon[(i + 1) % n]
    
    // 使用球面三角形面积计算（简化版）
    const lat1 = (p1.latitude * Math.PI) / 180
    const lat2 = (p2.latitude * Math.PI) / 180
    const lng1 = (p1.longitude * Math.PI) / 180
    const lng2 = (p2.longitude * Math.PI) / 180
    
    area += (lng2 - lng1) * (2 + Math.sin(lat1) + Math.sin(lat2))
  }
  
  area = Math.abs(area) * EARTH_RADIUS * EARTH_RADIUS / 2.0
  return area
}

/**
 * 参数验证
 */
export function validateParams(params: ObliqueParams): void {
  if (!params.polygonPoints || params.polygonPoints.length < 3) {
    throw new Error('作业区域多边形至少需要3个顶点')
  }
  if (params.sideOverlapRate < 0 || params.sideOverlapRate > 100) {
    throw new Error('旁路覆盖率必须在0-100%之间')
  }
  if (params.forwardOverlapRate < 0 || params.forwardOverlapRate > 100) {
    throw new Error('路线覆盖率必须在0-100%之间')
  }
  if (params.photoWidth <= 0 || params.photoLength <= 0) {
    throw new Error('照片尺寸必须大于0')
  }
  if (params.flightHeight <= 0) {
    throw new Error('飞行高度必须大于0')
  }
  if (params.gimbalPitch > 0) {
    throw new Error('云台俯仰角应为负值（向下）')
  }
}

/**
 * 分析航线优化策略
 */
export function analyzeRouteOptimization(params: ObliqueParams): RouteOptimizationResult {
  const pitch = Math.abs(params.gimbalPitch)
  
  // 云台俯仰角度不够大，使用垂直摄影
  if (pitch < Math.abs(MIN_EFFECTIVE_PITCH)) {
    return {
      directions: [params.mainDirection],
      gimbalPitches: [params.gimbalPitch],
      descriptions: ['垂直摄影航线（云台角度不足）'],
      isOptimized: true,
      reason: `云台俯仰角度小于${Math.abs(MIN_EFFECTIVE_PITCH)}度，采用单方向垂直摄影`
    }
  }

  // 云台角度适中，使用3航线十字交叉
  if (pitch >= Math.abs(MIN_EFFECTIVE_PITCH) && pitch < 30.0) {
    const directions = [
      params.mainDirection,
      (params.mainDirection + 90) % 360,
      (params.mainDirection + 180) % 360
    ]
    const pitches = [params.gimbalPitch, params.gimbalPitch, params.gimbalPitch]
    const descriptions = [
      '主方向倾斜摄影航线',
      '垂直方向倾斜摄影航线',
      '反向倾斜摄影航线'
    ]
    
    return {
      directions,
      gimbalPitches: pitches,
      descriptions,
      isOptimized: true,
      reason: `云台角度适中（${pitch}度），采用3方向十字交叉摄影提高效率`
    }
  }

  // 云台角度较大，使用4航线
  if (pitch >= 30.0 && pitch < 45.0) {
    const directions = [
      params.mainDirection,
      (params.mainDirection + 90) % 360,
      (params.mainDirection + 180) % 360,
      (params.mainDirection + 270) % 360
    ]
    const pitches = [params.gimbalPitch, params.gimbalPitch, params.gimbalPitch, params.gimbalPitch]
    const descriptions = [
      '主方向倾斜摄影航线',
      '东向倾斜摄影航线',
      '南向倾斜摄影航线',
      '西向倾斜摄影航线'
    ]
    
    return {
      directions,
      gimbalPitches: pitches,
      descriptions,
      isOptimized: true,
      reason: `云台角度较大（${pitch}度），采用4方向正交摄影保证覆盖质量`
    }
  }

  // 云台角度很大，需要完整的5航线
  const directions: number[] = []
  const pitches: number[] = []
  const descriptions: string[] = []
  
  for (let i = 0; i < 5; i++) {
    directions[i] = (params.mainDirection + STANDARD_5_DIRECTIONS[i]) % 360
    pitches[i] = params.gimbalPitch
    descriptions[i] = `第${i + 1}方向倾斜摄影航线（${directions[i].toFixed(0)}度）`
  }

  return {
    directions,
    gimbalPitches: pitches,
    descriptions,
    isOptimized: false,
    reason: `云台角度很大（${pitch}度），需要完整5方向摄影确保无死角覆盖`
  }
}

/**
 * 获取推荐的倾斜摄影参数
 */
export function getRecommendedGimbalPitch(targetHeight: number, cameraFOV: number): number {
  // 根据飞行高度和相机参数推荐合适的云台俯仰角
  if (targetHeight < 50) {
    return -20.0 // 低空飞行，较小俯仰角
  } else if (targetHeight < 100) {
    return -30.0 // 中等高度，中等俯仰角
  } else if (targetHeight < 200) {
    return -45.0 // 较高飞行，较大俯仰角
  } else {
    return -60.0 // 高空飞行，大俯仰角
  }
}

/**
 * 估算倾斜摄影作业时间
 */
export function estimateWorkTime(
  routes: { waypoints: LatLngPoint[]; distance: number }[], 
  totalDistance: number,
  cruiseSpeed: number, 
  photoInterval: number
): number {
  if (routes.length === 0) {
    return 0
  }

  const totalFlightTime = totalDistance / cruiseSpeed // 飞行时间（秒）
  
  // 估算总拍照点数
  let totalPhotoPoints = 0
  for (const route of routes) {
    totalPhotoPoints += route.waypoints.length
  }
  
  const totalPhotoTime = totalPhotoPoints * photoInterval // 拍照时间（秒）
  const routeChangeTime = (routes.length - 1) * 60 // 航线切换时间（秒）
  
  return (totalFlightTime + totalPhotoTime + routeChangeTime) / 60.0 // 转换为分钟
}