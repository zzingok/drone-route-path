/**
 * 倾斜摄影5航线算法 - 主要实现
 * 从 Java 版本转换而来
 * @author xhz
 */

import type { 
  LatLngPoint, 
  ObliqueParams, 
  ObliqueRouteResult,
  RouteDirection,
  ExpandedAreaInfo
} from '~/types/drone-route'

import {
  validateParams,
  analyzeRouteOptimization,
  calculateDistance,
  calculateDistanceFast,
  calculateRouteDistance,
  calculatePolygonArea,
  isPointInPolygon,
  isPointNearPolygon,
  fastCos,
  SpatialGrid
} from './drone-algorithm'

/**
 * 倾斜摄影的有效覆盖半径缓存
 */
const COVERAGE_RADIUS_CACHE = new Map<string, number>()

/**
 * 规划倾斜摄影5航线
 */
export async function planObliqueRoutes(params: ObliqueParams): Promise<ObliqueRouteResult> {
  // 参数验证
  validateParams(params)

  // 计算扩展区域以确保边缘覆盖
  const expandedPolygon = expandPolygonForEdgeCoverage(params)

  // 分析云台俯仰角，判断是否需要完整的5航线
  const optimization = analyzeRouteOptimization(params)

  const routes: RouteDirection[] = []
  let totalDistance = 0

  // 根据优化结果生成航线
  for (let i = 0; i < optimization.directions.length; i++) {
    const direction = optimization.directions[i]
    const gimbalPitch = optimization.gimbalPitches[i]
    const description = optimization.descriptions[i]

    // 使用扩展后的多边形进行航线规划
    const routeResult = await planRouteWithDirection(
      expandedPolygon,
      direction,
      params
    )

    if (routeResult.waypoints.length > 0) {
      // 确保航点能够完全覆盖原始多边形区域
      const effectiveWaypoints = ensureCompletePolygonCoverage(
        routeResult.waypoints, 
        params.polygonPoints, 
        params
      )
      
      if (effectiveWaypoints.length > 0) {
        // 重新计算距离
        const effectiveDistance = calculateRouteDistance(effectiveWaypoints)
        
        // 优化航点
        const optimizedWaypoints = optimizeWaypoints(effectiveWaypoints)
        
        const route: RouteDirection = {
          direction,
          gimbalPitch,
          waypoints: optimizedWaypoints,
          distance: effectiveDistance,
          description
        }
        
        routes.push(route)
        totalDistance += effectiveDistance
      }
    }
  }

  // 计算边缘覆盖率
  const edgeCoverageRate = calculateEdgeCoverageRate(routes, params.polygonPoints, params)
  
  // 计算扩展距离
  const expansionDistance = calculateExpansionDistance(params)
  
  const result: ObliqueRouteResult = {
    routes,
    totalDistance,
    totalRouteCount: routes.length,
    isOptimized: optimization.isOptimized,
    optimizationReason: optimization.reason,
    expandedPolygon,
    expansionDistance,
    edgeCoverageRate
  }
  
  return result
}

/**
 * 扩展多边形以确保边缘覆盖
 */
function expandPolygonForEdgeCoverage(params: ObliqueParams): LatLngPoint[] {
  const expansionDistance = calculateExpansionDistance(params)
  return expandPolygon(params.polygonPoints, expansionDistance)
}

/**
 * 向外扩展多边形
 */
function expandPolygon(originalPolygon: LatLngPoint[], expansionDistance: number): LatLngPoint[] {
  if (originalPolygon.length < 3) {
    return [...originalPolygon]
  }
  
  const expandedPolygon: LatLngPoint[] = []
  const n = originalPolygon.length
  
  // 确保多边形是逆时针方向
  const polygon = ensureCounterClockwise(originalPolygon)
  
  for (let i = 0; i < n; i++) {
    const current = polygon[i]
    const prev = polygon[(i - 1 + n) % n]
    const next = polygon[(i + 1) % n]
    
    // 计算当前点的向外法向量
    const outwardNormal = calculateOutwardNormal(prev, current, next)
    
    // 将扩展距离转换为经纬度偏移
    const offset = convertDistanceToLatLngOffset(current, expansionDistance, outwardNormal)
    
    const expandedPoint: LatLngPoint = {
      latitude: current.latitude + offset[0],
      longitude: current.longitude + offset[1]
    }
    
    expandedPolygon.push(expandedPoint)
  }
  
  return expandedPolygon
}

/**
 * 确保多边形是逆时针方向
 */
function ensureCounterClockwise(polygon: LatLngPoint[]): LatLngPoint[] {
  if (polygon.length < 3) {
    return [...polygon]
  }
  
  // 计算多边形的有向面积
  let signedArea = 0.0
  const n = polygon.length
  
  for (let i = 0; i < n; i++) {
    const p1 = polygon[i]
    const p2 = polygon[(i + 1) % n]
    signedArea += (p2.longitude - p1.longitude) * (p2.latitude + p1.latitude)
  }
  
  // 如果有向面积为正，说明是顺时针，需要反转
  if (signedArea > 0) {
    return [...polygon].reverse()
  }
  
  return [...polygon]
}

/**
 * 计算向外法向量
 */
function calculateOutwardNormal(prev: LatLngPoint, current: LatLngPoint, next: LatLngPoint): [number, number] {
  // 计算两个相邻边的向量
  const edge1Lat = current.latitude - prev.latitude
  const edge1Lng = current.longitude - prev.longitude
  const edge2Lat = next.latitude - current.latitude
  const edge2Lng = next.longitude - current.longitude
  
  // 计算边的法向量（向右旋转90度）
  const normal1Lat = -edge1Lng
  const normal1Lng = edge1Lat
  const normal2Lat = -edge2Lng
  const normal2Lng = edge2Lat
  
  // 计算角平分线方向
  const bisectorLat = normal1Lat + normal2Lat
  const bisectorLng = normal1Lng + normal2Lng
  
  // 归一化
  const length = Math.sqrt(bisectorLat * bisectorLat + bisectorLng * bisectorLng)
  if (length > 1e-10) {
    return [bisectorLat / length, bisectorLng / length]
  } else {
    // 如果角平分线长度为0（180度角），使用垂直于第一条边的向量
    const length1 = Math.sqrt(normal1Lat * normal1Lat + normal1Lng * normal1Lng)
    if (length1 > 1e-10) {
      return [normal1Lat / length1, normal1Lng / length1]
    } else {
      return [0, 0]
    }
  }
}

/**
 * 将距离转换为经纬度偏移
 */
function convertDistanceToLatLngOffset(point: LatLngPoint, distance: number, direction: [number, number]): [number, number] {
  // 纬度偏移（1度纬度约等于111000米）
  const latOffset = (distance * direction[0]) / 111000.0
  
  // 经度偏移（考虑纬度的影响）
  const cosLat = Math.cos((point.latitude * Math.PI) / 180)
  const lngOffset = (distance * direction[1]) / (111000.0 * cosLat)
  
  return [latOffset, lngOffset]
}

/**
 * 计算扩展距离
 */
function calculateExpansionDistance(params: ObliqueParams): number {
  const gimbalPitchRad = (Math.abs(params.gimbalPitch) * Math.PI) / 180
  
  // 照片的地面覆盖尺寸
  const maxGroundCoverage = Math.max(params.photoWidth, params.photoLength)
  
  // 计算倾斜摄影的实际地面投影偏移
  let tiltOffset = 0
  if (Math.abs(params.gimbalPitch) > 5.0) { // 只有在明显倾斜时才考虑偏移
    tiltOffset = params.flightHeight * Math.tan(gimbalPitchRad) * 0.5 // 减小偏移影响
  }
  
  // 考虑重叠率影响
  const minOverlapRate = Math.min(params.sideOverlapRate, params.forwardOverlapRate) / 100.0
  const overlapFactor = 1.0 - minOverlapRate * 0.1 // 减小重叠率的影响
  
  // 基础扩展距离：主要基于照片覆盖范围
  const baseExpansion = maxGroundCoverage * 0.6 // 减小基础扩展系数
  const expansionDistance = (baseExpansion + tiltOffset) * overlapFactor
  
  // 设置更合理的扩展距离范围
  const minExpansion = maxGroundCoverage * 0.3 // 最小扩展距离
  const maxExpansion = maxGroundCoverage * 0.8 + tiltOffset // 最大扩展距离
  
  return Math.max(minExpansion, Math.min(expansionDistance, maxExpansion))
}

/**
 * 简化的航线规划（模拟RoutePathPlanner.planRouteWithMultipleBlocks）
 */
async function planRouteWithDirection(
  polygon: LatLngPoint[],
  direction: number,
  params: ObliqueParams
): Promise<{ waypoints: LatLngPoint[] }> {
  // 这里简化实现，实际应该调用完整的航线规划算法
  const waypoints: LatLngPoint[] = []
  
  // 计算多边形边界框
  const bounds = getPolygonBounds(polygon)
  
  // 计算航线间距
  const lineSpacing = params.photoWidth * (1 - params.sideOverlapRate / 100)
  
  // 根据方向生成平行航线
  const lines = generateParallelLines(bounds, direction, lineSpacing)
  
  // 与多边形求交，生成航点
  for (const line of lines) {
    const intersections = getLinePolygonIntersections(line, polygon)
    waypoints.push(...intersections)
  }
  
  return { waypoints }
}

/**
 * 获取多边形边界框
 */
function getPolygonBounds(polygon: LatLngPoint[]): {
  minLat: number
  maxLat: number
  minLng: number
  maxLng: number
} {
  let minLat = Number.MAX_VALUE
  let maxLat = Number.MIN_VALUE
  let minLng = Number.MAX_VALUE
  let maxLng = Number.MIN_VALUE
  
  for (const point of polygon) {
    minLat = Math.min(minLat, point.latitude)
    maxLat = Math.max(maxLat, point.latitude)
    minLng = Math.min(minLng, point.longitude)
    maxLng = Math.max(maxLng, point.longitude)
  }
  
  return { minLat, maxLat, minLng, maxLng }
}

/**
 * 生成平行航线（简化版）
 */
function generateParallelLines(
  bounds: { minLat: number; maxLat: number; minLng: number; maxLng: number },
  direction: number,
  spacing: number
): Array<{ start: LatLngPoint; end: LatLngPoint }> {
  const lines: Array<{ start: LatLngPoint; end: LatLngPoint }> = []
  
  // 简化实现：生成覆盖边界框的平行线
  const directionRad = (direction * Math.PI) / 180
  const perpDirection = direction + 90
  const perpDirectionRad = (perpDirection * Math.PI) / 180
  
  // 计算需要的航线数量
  const centerLat = (bounds.minLat + bounds.maxLat) / 2
  const centerLng = (bounds.minLng + bounds.maxLng) / 2
  const width = Math.max(bounds.maxLat - bounds.minLat, bounds.maxLng - bounds.minLng) * 111000
  const lineCount = Math.ceil(width / spacing) + 2
  
  for (let i = 0; i < lineCount; i++) {
    const offset = (i - lineCount / 2) * spacing
    const offsetLat = (offset * Math.cos(perpDirectionRad)) / 111000
    const offsetLng = (offset * Math.sin(perpDirectionRad)) / (111000 * Math.cos((centerLat * Math.PI) / 180))
    
    const lineCenter: LatLngPoint = {
      latitude: centerLat + offsetLat,
      longitude: centerLng + offsetLng
    }
    
    // 创建足够长的线段
    const lineLength = width * 2
    const halfLengthLat = (lineLength * Math.cos(directionRad)) / 111000 / 2
    const halfLengthLng = (lineLength * Math.sin(directionRad)) / (111000 * Math.cos((centerLat * Math.PI) / 180)) / 2
    
    lines.push({
      start: {
        latitude: lineCenter.latitude - halfLengthLat,
        longitude: lineCenter.longitude - halfLengthLng
      },
      end: {
        latitude: lineCenter.latitude + halfLengthLat,
        longitude: lineCenter.longitude + halfLengthLng
      }
    })
  }
  
  return lines
}

/**
 * 计算直线与多边形的交点（简化版）
 */
function getLinePolygonIntersections(
  line: { start: LatLngPoint; end: LatLngPoint },
  polygon: LatLngPoint[]
): LatLngPoint[] {
  const intersections: LatLngPoint[] = []
  
  // 简化实现：只返回线段与多边形内部的交点
  const steps = 50 // 将线段分为50步进行采样
  
  for (let i = 0; i <= steps; i++) {
    const t = i / steps
    const point: LatLngPoint = {
      latitude: line.start.latitude + t * (line.end.latitude - line.start.latitude),
      longitude: line.start.longitude + t * (line.end.longitude - line.start.longitude)
    }
    
    if (isPointInPolygon(point, polygon)) {
      intersections.push(point)
    }
  }
  
  // 去重和排序
  return intersections.filter((point, index) => {
    if (index === 0) return true
    const prev = intersections[index - 1]
    return calculateDistanceFast(point, prev) > 10 // 10米最小距离
  })
}

/**
 * 确保航点能够完全覆盖原始多边形区域
 */
function ensureCompletePolygonCoverage(
  waypoints: LatLngPoint[], 
  originalPolygon: LatLngPoint[], 
  params: ObliqueParams
): LatLngPoint[] {
  const effectiveWaypoints = new Set<LatLngPoint>()
  
  // 计算有效覆盖半径
  const coverageRadius = calculateEffectiveCoverageRadius(params)
  
  // 1. 首先保留所有能够覆盖原始多边形的航点
  for (const waypoint of waypoints) {
    if (canWaypointCoverPolygonArea(waypoint, originalPolygon, coverageRadius)) {
      effectiveWaypoints.add(waypoint)
    }
  }
  
  // 2. 检查原始多边形的覆盖完整性
  const uncoveredAreas = findUncoveredAreas(originalPolygon, Array.from(effectiveWaypoints), coverageRadius)
  
  // 3. 如果有未覆盖区域，从原始航点中补充
  if (uncoveredAreas.length > 0) {
    for (const waypoint of waypoints) {
      if (!effectiveWaypoints.has(waypoint)) {
        // 检查这个航点是否能覆盖未覆盖的区域
        const coversUncoveredArea = uncoveredAreas.some(uncoveredPoint => 
          calculateDistanceFast(waypoint, uncoveredPoint) <= coverageRadius
        )
        
        if (coversUncoveredArea) {
          effectiveWaypoints.add(waypoint)
        }
      }
    }
  }
  
  return Array.from(effectiveWaypoints)
}

/**
 * 计算倾斜摄影的有效覆盖半径
 */
function calculateEffectiveCoverageRadius(params: ObliqueParams): number {
  const key = `${params.photoWidth.toFixed(2)},${params.photoLength.toFixed(2)},${params.gimbalPitch.toFixed(2)},${params.sideOverlapRate.toFixed(2)},${params.forwardOverlapRate.toFixed(2)}`
  
  if (!COVERAGE_RADIUS_CACHE.has(key)) {
    // 基础覆盖半径
    const baseRadius = Math.max(params.photoWidth, params.photoLength) * 0.5
    
    // 考虑倾斜角度的影响
    const tiltFactor = fastCos(Math.abs(params.gimbalPitch)) // 倾斜角度越大，有效覆盖面积越小
    
    // 考虑重叠率的影响
    const overlapFactor = Math.min(params.sideOverlapRate, params.forwardOverlapRate) / 100.0
    const effectiveRadius = baseRadius * tiltFactor * (1 - overlapFactor * 0.3)
    
    // 确保最小覆盖半径
    const result = Math.max(effectiveRadius, baseRadius * 0.4)
    COVERAGE_RADIUS_CACHE.set(key, result)
  }
  
  return COVERAGE_RADIUS_CACHE.get(key)!
}

/**
 * 检查航点是否能够覆盖多边形区域
 */
function canWaypointCoverPolygonArea(waypoint: LatLngPoint, polygon: LatLngPoint[], coverageRadius: number): boolean {
  // 先检查是否在多边形内部（最快的情况）
  if (isPointInPolygon(waypoint, polygon)) {
    return true
  }
  
  // 详细的边界距离检查
  return isPointNearPolygon(waypoint, polygon, coverageRadius)
}

/**
 * 找出原始多边形中未被覆盖的区域
 */
function findUncoveredAreas(
  originalPolygon: LatLngPoint[], 
  waypoints: LatLngPoint[], 
  coverageRadius: number
): LatLngPoint[] {
  // 在多边形内部进行网格采样，检查覆盖情况
  const samplePoints = samplePolygonInterior(originalPolygon, coverageRadius * 0.5)
  
  return samplePoints.filter(samplePoint => {
    return !waypoints.some(waypoint => 
      calculateDistanceFast(samplePoint, waypoint) <= coverageRadius
    )
  })
}

/**
 * 在多边形内部进行采样
 */
function samplePolygonInterior(polygon: LatLngPoint[], sampleDistance: number): LatLngPoint[] {
  const samplePoints: LatLngPoint[] = []
  
  // 计算多边形边界框
  const bounds = getPolygonBounds(polygon)
  
  // 转换采样距离为经纬度增量
  const latStep = sampleDistance / 111000.0
  const avgLat = (bounds.minLat + bounds.maxLat) / 2
  const lngStep = sampleDistance / (111000.0 * Math.cos((avgLat * Math.PI) / 180))
  
  // 在边界框内进行网格采样
  for (let lat = bounds.minLat; lat <= bounds.maxLat; lat += latStep) {
    for (let lng = bounds.minLng; lng <= bounds.maxLng; lng += lngStep) {
      const samplePoint: LatLngPoint = { latitude: lat, longitude: lng }
      if (isPointInPolygon(samplePoint, polygon)) {
        samplePoints.push(samplePoint)
      }
    }
  }
  
  // 同时在多边形边界上采样
  samplePoints.push(...samplePolygonEdge(polygon, sampleDistance))
  
  return samplePoints
}

/**
 * 在多边形边界上采样点
 */
function samplePolygonEdge(polygon: LatLngPoint[], sampleDistance: number): LatLngPoint[] {
  const samplePoints: LatLngPoint[] = []
  
  for (let i = 0; i < polygon.length; i++) {
    const p1 = polygon[i]
    const p2 = polygon[(i + 1) % polygon.length]
    
    const edgeLength = calculateDistance(p1, p2)
    const sampleCount = Math.max(1, Math.ceil(edgeLength / sampleDistance))
    
    for (let j = 0; j < sampleCount; j++) {
      const ratio = j / sampleCount
      const samplePoint: LatLngPoint = {
        latitude: p1.latitude + (p2.latitude - p1.latitude) * ratio,
        longitude: p1.longitude + (p2.longitude - p1.longitude) * ratio
      }
      samplePoints.push(samplePoint)
    }
  }
  
  return samplePoints
}

/**
 * 优化航点（简化版）
 */
function optimizeWaypoints(waypoints: LatLngPoint[]): LatLngPoint[] {
  // 简单的优化：去除过于接近的点
  const optimized: LatLngPoint[] = []
  const minDistance = 5 // 最小5米距离
  
  for (const waypoint of waypoints) {
    const isClose = optimized.some(existing => 
      calculateDistanceFast(waypoint, existing) < minDistance
    )
    
    if (!isClose) {
      optimized.push(waypoint)
    }
  }
  
  return optimized
}

/**
 * 计算边缘覆盖率
 */
function calculateEdgeCoverageRate(routes: RouteDirection[], originalPolygon: LatLngPoint[], params: ObliqueParams): number {
  if (routes.length === 0) {
    return 0.0
  }
  
  // 收集所有航点
  const allWaypoints: LatLngPoint[] = []
  for (const route of routes) {
    allWaypoints.push(...route.waypoints)
  }
  
  // 计算覆盖半径
  const coverageRadius = Math.max(params.photoWidth, params.photoLength) * 0.6
  
  // 在多边形边界上采样点
  const edgeSamplePoints = samplePolygonEdge(originalPolygon, 10.0) // 每10米采样一个点
  
  let coveredPoints = 0
  for (const edgePoint of edgeSamplePoints) {
    const covered = allWaypoints.some(waypoint => 
      calculateDistance(edgePoint, waypoint) <= coverageRadius
    )
    
    if (covered) {
      coveredPoints++
    }
  }
  
  return edgeSamplePoints.length === 0 ? 100.0 : (coveredPoints / edgeSamplePoints.length) * 100.0
}

/**
 * 获取扩展后的作业区域信息
 */
export function getExpandedAreaInfo(params: ObliqueParams): ExpandedAreaInfo {
  validateParams(params)
  
  const expandedPolygon = expandPolygonForEdgeCoverage(params)
  const expansionDistance = calculateExpansionDistance(params)
  const originalArea = calculatePolygonArea(params.polygonPoints)
  const expandedArea = calculatePolygonArea(expandedPolygon)
  
  return {
    expandedPolygon,
    expansionDistance,
    originalArea,
    expandedArea,
    areaIncreasePercentage: (expandedArea - originalArea) / originalArea * 100
  }
}