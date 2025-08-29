/**
 * Vue 3 Composable for 倾斜摄影5航线算法
 * 从 Java 版本转换而来
 * @author xhz
 */

import { ref, computed, reactive } from 'vue'
import type { 
  LatLngPoint, 
  ObliqueParams, 
  ObliqueRouteResult,
  ExpandedAreaInfo 
} from '~/types/drone-route'

import { 
  planObliqueRoutes, 
  getExpandedAreaInfo 
} from '~/utils/oblique-photography-algorithm'

import { 
  getRecommendedGimbalPitch, 
  estimateWorkTime,
  validateParams 
} from '~/utils/drone-algorithm'

export const useObliquePhotography = () => {
  // 响应式状态
  const isLoading = ref(false)
  const error = ref<string | null>(null)
  const result = ref<ObliqueRouteResult | null>(null)
  const expandedAreaInfo = ref<ExpandedAreaInfo | null>(null)

  // 默认参数
  const defaultParams = reactive<ObliqueParams>({
    mainDirection: 0,          // 主航线方向（0-360度）
    sideOverlapRate: 80,       // 旁路覆盖率（80%）
    forwardOverlapRate: 80,    // 路线覆盖率（80%）
    photoWidth: 50,           // 照片宽度（50米）
    photoLength: 50,          // 照片长度（50米）
    gimbalPitch: -30,         // 云台俯仰角（-30度）
    flightHeight: 100,        // 飞行高度（100米）
    polygonPoints: [],        // 作业区域多边形顶点
    startPoint: { latitude: 0, longitude: 0 } // 起始点
  })

  // 计算属性
  const hasValidPolygon = computed(() => {
    return defaultParams.polygonPoints.length >= 3
  })

  const estimatedWorkTime = computed(() => {
    if (!result.value) return 0
    return estimateWorkTime(
      result.value.routes,
      result.value.totalDistance,
      15, // 默认巡航速度 15m/s
      2   // 默认拍照间隔 2秒
    )
  })

  const recommendedGimbalPitch = computed(() => {
    return getRecommendedGimbalPitch(defaultParams.flightHeight, 84) // 默认相机视场角84度
  })

  // 核心方法

  /**
   * 规划倾斜摄影航线
   */
  const planRoutes = async (params?: Partial<ObliqueParams>) => {
    isLoading.value = true
    error.value = null
    result.value = null

    try {
      // 合并参数
      const finalParams: ObliqueParams = { ...defaultParams, ...params }
      
      // 参数验证
      validateParams(finalParams)

      // 执行算法
      const routeResult = await planObliqueRoutes(finalParams)
      result.value = routeResult

      // 计算扩展区域信息
      const areaInfo = getExpandedAreaInfo(finalParams)
      expandedAreaInfo.value = areaInfo

      return routeResult
    } catch (err) {
      error.value = err instanceof Error ? err.message : '规划航线时发生未知错误'
      throw err
    } finally {
      isLoading.value = false
    }
  }

  /**
   * 更新参数
   */
  const updateParams = (params: Partial<ObliqueParams>) => {
    Object.assign(defaultParams, params)
  }

  /**
   * 设置作业区域多边形
   */
  const setPolygon = (points: LatLngPoint[]) => {
    if (points.length < 3) {
      throw new Error('多边形至少需要3个顶点')
    }
    defaultParams.polygonPoints = [...points]
  }

  /**
   * 添加多边形顶点
   */
  const addPolygonPoint = (point: LatLngPoint) => {
    defaultParams.polygonPoints.push({ ...point })
  }

  /**
   * 移除多边形顶点
   */
  const removePolygonPoint = (index: number) => {
    if (index >= 0 && index < defaultParams.polygonPoints.length) {
      defaultParams.polygonPoints.splice(index, 1)
    }
  }

  /**
   * 清空多边形
   */
  const clearPolygon = () => {
    defaultParams.polygonPoints = []
  }

  /**
   * 设置起始点
   */
  const setStartPoint = (point: LatLngPoint) => {
    defaultParams.startPoint = { ...point }
  }

  /**
   * 重置所有参数为默认值
   */
  const resetParams = () => {
    Object.assign(defaultParams, {
      mainDirection: 0,
      sideOverlapRate: 80,
      forwardOverlapRate: 80,
      photoWidth: 50,
      photoLength: 50,
      gimbalPitch: -30,
      flightHeight: 100,
      polygonPoints: [],
      startPoint: { latitude: 0, longitude: 0 }
    })
    result.value = null
    expandedAreaInfo.value = null
    error.value = null
  }

  /**
   * 导出结果为GeoJSON格式
   */
  const exportToGeoJSON = () => {
    if (!result.value) {
      throw new Error('没有可导出的航线数据')
    }

    const features = []

    // 添加原始多边形
    features.push({
      type: 'Feature',
      properties: {
        name: '作业区域',
        type: 'workArea'
      },
      geometry: {
        type: 'Polygon',
        coordinates: [defaultParams.polygonPoints.map(p => [p.longitude, p.latitude])]
      }
    })

    // 添加扩展多边形
    if (result.value.expandedPolygon) {
      features.push({
        type: 'Feature',
        properties: {
          name: '扩展作业区域',
          type: 'expandedArea'
        },
        geometry: {
          type: 'Polygon',
          coordinates: [result.value.expandedPolygon.map(p => [p.longitude, p.latitude])]
        }
      })
    }

    // 添加各方向航线
    result.value.routes.forEach((route, index) => {
      features.push({
        type: 'Feature',
        properties: {
          name: route.description,
          direction: route.direction,
          gimbalPitch: route.gimbalPitch,
          distance: route.distance,
          routeIndex: index,
          type: 'route'
        },
        geometry: {
          type: 'LineString',
          coordinates: route.waypoints.map(p => [p.longitude, p.latitude])
        }
      })

      // 添加航点
      route.waypoints.forEach((waypoint, wpIndex) => {
        features.push({
          type: 'Feature',
          properties: {
            name: `航点-${index}-${wpIndex}`,
            routeIndex: index,
            waypointIndex: wpIndex,
            type: 'waypoint'
          },
          geometry: {
            type: 'Point',
            coordinates: [waypoint.longitude, waypoint.latitude]
          }
        })
      })
    })

    return {
      type: 'FeatureCollection',
      properties: {
        name: '倾斜摄影航线规划',
        totalDistance: result.value.totalDistance,
        totalRouteCount: result.value.totalRouteCount,
        isOptimized: result.value.isOptimized,
        optimizationReason: result.value.optimizationReason,
        expansionDistance: result.value.expansionDistance,
        edgeCoverageRate: result.value.edgeCoverageRate,
        estimatedWorkTime: estimatedWorkTime.value,
        generatedAt: new Date().toISOString()
      },
      features
    }
  }

  /**
   * 从GeoJSON导入多边形
   */
  const importFromGeoJSON = (geoJSON: any) => {
    try {
      if (geoJSON.type === 'FeatureCollection') {
        // 查找多边形特征
        const polygonFeature = geoJSON.features.find((f: any) => 
          f.geometry.type === 'Polygon' && f.properties?.type === 'workArea'
        )
        
        if (polygonFeature) {
          const coordinates = polygonFeature.geometry.coordinates[0]
          const points: LatLngPoint[] = coordinates.map((coord: number[]) => ({
            latitude: coord[1],
            longitude: coord[0]
          }))
          setPolygon(points)
        }
      } else if (geoJSON.type === 'Feature' && geoJSON.geometry.type === 'Polygon') {
        const coordinates = geoJSON.geometry.coordinates[0]
        const points: LatLngPoint[] = coordinates.map((coord: number[]) => ({
          latitude: coord[1],
          longitude: coord[0]
        }))
        setPolygon(points)
      }
    } catch (err) {
      throw new Error('导入GeoJSON格式错误')
    }
  }

  return {
    // 状态
    isLoading: readonly(isLoading),
    error: readonly(error),
    result: readonly(result),
    expandedAreaInfo: readonly(expandedAreaInfo),
    
    // 参数
    params: defaultParams,
    
    // 计算属性
    hasValidPolygon,
    estimatedWorkTime,
    recommendedGimbalPitch,
    
    // 方法
    planRoutes,
    updateParams,
    setPolygon,
    addPolygonPoint,
    removePolygonPoint,
    clearPolygon,
    setStartPoint,
    resetParams,
    exportToGeoJSON,
    importFromGeoJSON
  }
}

// 预设参数配置
export const presetConfigs = {
  // 低空精细摄影
  lowAltitudeDetailed: {
    flightHeight: 50,
    gimbalPitch: -20,
    sideOverlapRate: 85,
    forwardOverlapRate: 85,
    photoWidth: 30,
    photoLength: 30
  },
  
  // 中等高度标准摄影
  mediumAltitudeStandard: {
    flightHeight: 100,
    gimbalPitch: -30,
    sideOverlapRate: 80,
    forwardOverlapRate: 80,
    photoWidth: 50,
    photoLength: 50
  },
  
  // 高空大范围摄影
  highAltitudeWide: {
    flightHeight: 200,
    gimbalPitch: -45,
    sideOverlapRate: 75,
    forwardOverlapRate: 75,
    photoWidth: 100,
    photoLength: 100
  },
  
  // 超高精度摄影
  ultraHighPrecision: {
    flightHeight: 30,
    gimbalPitch: -15,
    sideOverlapRate: 90,
    forwardOverlapRate: 90,
    photoWidth: 20,
    photoLength: 20
  }
}