<template>
  <div class="oblique-photography-planner">
    <!-- Header -->
    <div class="planner-header">
      <h1 class="title">倾斜摄影5航线算法</h1>
      <p class="subtitle">智能无人机航线规划系统</p>
    </div>

    <!-- Parameter Input Section -->
    <div class="parameter-section">
      <h2>参数设置</h2>
      
      <!-- Preset Configurations -->
      <div class="preset-section">
        <label>预设配置:</label>
        <div class="preset-buttons">
          <button
            v-for="(config, key) in presetConfigs"
            :key="key"
            @click="applyPreset(key)"
            class="preset-btn"
          >
            {{ getPresetName(key) }}
          </button>
        </div>
      </div>

      <!-- Flight Parameters -->
      <div class="parameter-group">
        <h3>飞行参数</h3>
        <div class="input-grid">
          <div class="input-field">
            <label>主航线方向 (度)</label>
            <input
              v-model.number="params.mainDirection"
              type="number"
              min="0"
              max="360"
              step="1"
            />
          </div>
          
          <div class="input-field">
            <label>飞行高度 (米)</label>
            <input
              v-model.number="params.flightHeight"
              type="number"
              min="10"
              max="500"
              step="1"
            />
          </div>
          
          <div class="input-field">
            <label>云台俯仰角 (度)</label>
            <input
              v-model.number="params.gimbalPitch"
              type="number"
              min="-90"
              max="0"
              step="1"
            />
            <small>推荐值: {{ recommendedGimbalPitch }}°</small>
          </div>
        </div>
      </div>

      <!-- Photo Parameters -->
      <div class="parameter-group">
        <h3>拍摄参数</h3>
        <div class="input-grid">
          <div class="input-field">
            <label>照片宽度 (米)</label>
            <input
              v-model.number="params.photoWidth"
              type="number"
              min="1"
              max="1000"
              step="1"
            />
          </div>
          
          <div class="input-field">
            <label>照片长度 (米)</label>
            <input
              v-model.number="params.photoLength"
              type="number"
              min="1"
              max="1000"
              step="1"
            />
          </div>
          
          <div class="input-field">
            <label>旁路覆盖率 (%)</label>
            <input
              v-model.number="params.sideOverlapRate"
              type="number"
              min="0"
              max="100"
              step="1"
            />
          </div>
          
          <div class="input-field">
            <label>前向覆盖率 (%)</label>
            <input
              v-model.number="params.forwardOverlapRate"
              type="number"
              min="0"
              max="100"
              step="1"
            />
          </div>
        </div>
      </div>

      <!-- Work Area Definition -->
      <div class="parameter-group">
        <h3>作业区域</h3>
        
        <!-- Start Point -->
        <div class="input-grid">
          <div class="input-field">
            <label>起始点纬度</label>
            <input
              v-model.number="params.startPoint.latitude"
              type="number"
              step="0.000001"
            />
          </div>
          
          <div class="input-field">
            <label>起始点经度</label>
            <input
              v-model.number="params.startPoint.longitude"
              type="number"
              step="0.000001"
            />
          </div>
        </div>

        <!-- Polygon Points -->
        <div class="polygon-section">
          <div class="polygon-header">
            <label>多边形顶点 ({{ params.polygonPoints.length }} 个点)</label>
            <button @click="addSamplePolygon" class="btn-secondary">
              添加示例多边形
            </button>
          </div>
          
          <div class="polygon-points">
            <div
              v-for="(point, index) in params.polygonPoints"
              :key="index"
              class="point-row"
            >
              <span class="point-index">{{ index + 1 }}.</span>
              <input
                v-model.number="point.latitude"
                type="number"
                step="0.000001"
                placeholder="纬度"
              />
              <input
                v-model.number="point.longitude"
                type="number"
                step="0.000001"
                placeholder="经度"
              />
              <button
                @click="removePolygonPoint(index)"
                class="btn-danger"
              >
                删除
              </button>
            </div>
          </div>
          
          <div class="polygon-actions">
            <button @click="addEmptyPoint" class="btn-secondary">
              添加顶点
            </button>
            <button @click="clearPolygon" class="btn-secondary">
              清空多边形
            </button>
          </div>
        </div>
      </div>

      <!-- Action Buttons -->
      <div class="action-section">
        <button
          @click="planRoutes"
          :disabled="!hasValidPolygon || isLoading"
          class="btn-primary plan-btn"
        >
          <span v-if="isLoading">规划中...</span>
          <span v-else>规划航线</span>
        </button>
        
        <button
          @click="resetParams"
          class="btn-secondary"
        >
          重置参数
        </button>
      </div>
    </div>

    <!-- Error Display -->
    <div v-if="error" class="error-section">
      <div class="error-message">
        <strong>错误:</strong> {{ error }}
      </div>
    </div>

    <!-- Results Section -->
    <div v-if="result" class="results-section">
      <h2>规划结果</h2>
      
      <!-- Summary -->
      <div class="result-summary">
        <div class="summary-card">
          <h3>基本信息</h3>
          <div class="stats">
            <div class="stat">
              <label>航线数量:</label>
              <span>{{ result.totalRouteCount }} 条</span>
            </div>
            <div class="stat">
              <label>总距离:</label>
              <span>{{ formatDistance(result.totalDistance) }}</span>
            </div>
            <div class="stat">
              <label>预估时间:</label>
              <span>{{ formatTime(estimatedWorkTime) }}</span>
            </div>
            <div class="stat">
              <label>边缘覆盖率:</label>
              <span>{{ result.edgeCoverageRate.toFixed(1) }}%</span>
            </div>
          </div>
        </div>

        <div class="summary-card">
          <h3>优化信息</h3>
          <div class="optimization-info">
            <div class="optimization-status">
              <span :class="['status', { optimized: result.isOptimized }]">
                {{ result.isOptimized ? '已优化' : '标准模式' }}
              </span>
            </div>
            <p class="optimization-reason">{{ result.optimizationReason }}</p>
          </div>
        </div>

        <div v-if="expandedAreaInfo" class="summary-card">
          <h3>区域扩展</h3>
          <div class="stats">
            <div class="stat">
              <label>扩展距离:</label>
              <span>{{ expandedAreaInfo.expansionDistance.toFixed(1) }} 米</span>
            </div>
            <div class="stat">
              <label>原始面积:</label>
              <span>{{ formatArea(expandedAreaInfo.originalArea) }}</span>
            </div>
            <div class="stat">
              <label>扩展面积:</label>
              <span>{{ formatArea(expandedAreaInfo.expandedArea) }}</span>
            </div>
            <div class="stat">
              <label>面积增加:</label>
              <span>{{ expandedAreaInfo.areaIncreasePercentage.toFixed(1) }}%</span>
            </div>
          </div>
        </div>
      </div>

      <!-- Route Details -->
      <div class="route-details">
        <h3>航线详情</h3>
        <div class="route-list">
          <div
            v-for="(route, index) in result.routes"
            :key="index"
            class="route-item"
          >
            <div class="route-header">
              <h4>{{ route.description }}</h4>
              <div class="route-stats">
                <span>方向: {{ route.direction.toFixed(0) }}°</span>
                <span>俯仰角: {{ route.gimbalPitch.toFixed(0) }}°</span>
                <span>距离: {{ formatDistance(route.distance) }}</span>
                <span>航点: {{ route.waypoints.length }} 个</span>
              </div>
            </div>
            
            <div class="waypoints-section" v-if="showWaypoints[index]">
              <div class="waypoints-header">
                <span>航点坐标</span>
              </div>
              <div class="waypoints-list">
                <div
                  v-for="(waypoint, wpIndex) in route.waypoints"
                  :key="wpIndex"
                  class="waypoint-item"
                >
                  <span class="waypoint-index">{{ wpIndex + 1 }}</span>
                  <span class="coordinates">
                    {{ waypoint.latitude.toFixed(6) }}, {{ waypoint.longitude.toFixed(6) }}
                  </span>
                </div>
              </div>
            </div>
            
            <button
              @click="toggleWaypoints(index)"
              class="btn-secondary waypoints-toggle"
            >
              {{ showWaypoints[index] ? '隐藏航点' : '显示航点' }}
            </button>
          </div>
        </div>
      </div>

      <!-- Export Section -->
      <div class="export-section">
        <h3>导出选项</h3>
        <div class="export-buttons">
          <button @click="exportGeoJSON" class="btn-secondary">
            导出 GeoJSON
          </button>
          <button @click="exportCSV" class="btn-secondary">
            导出 CSV
          </button>
          <button @click="exportSummary" class="btn-secondary">
            导出摘要
          </button>
        </div>
      </div>
    </div>
  </div>
</template>

<script setup lang="ts">
import { ref, reactive } from 'vue'
import { useObliquePhotography, presetConfigs } from '~/composables/useObliquePhotography'

// 使用组合式API
const {
  isLoading,
  error,
  result,
  expandedAreaInfo,
  params,
  hasValidPolygon,
  estimatedWorkTime,
  recommendedGimbalPitch,
  planRoutes: planRoutesComposable,
  addPolygonPoint,
  removePolygonPoint,
  clearPolygon,
  resetParams: resetParamsComposable,
  exportToGeoJSON,
  updateParams
} = useObliquePhotography()

// 响应式状态
const showWaypoints = ref<Record<number, boolean>>({})

// 方法
const planRoutes = async () => {
  try {
    await planRoutesComposable()
    // 重置航点显示状态
    showWaypoints.value = {}
  } catch (err) {
    console.error('规划航线失败:', err)
  }
}

const resetParams = () => {
  resetParamsComposable()
  showWaypoints.value = {}
}

const applyPreset = (presetKey: string) => {
  const config = presetConfigs[presetKey as keyof typeof presetConfigs]
  if (config) {
    updateParams(config)
  }
}

const getPresetName = (key: string): string => {
  const names: Record<string, string> = {
    lowAltitudeDetailed: '低空精细',
    mediumAltitudeStandard: '中等标准',
    highAltitudeWide: '高空大范围',
    ultraHighPrecision: '超高精度'
  }
  return names[key] || key
}

const addEmptyPoint = () => {
  addPolygonPoint({ latitude: 0, longitude: 0 })
}

const addSamplePolygon = () => {
  // 添加示例多边形（北京某区域）
  const samplePoints = [
    { latitude: 39.9042, longitude: 116.4074 },
    { latitude: 39.9042, longitude: 116.4174 },
    { latitude: 39.8942, longitude: 116.4174 },
    { latitude: 39.8942, longitude: 116.4074 }
  ]
  
  clearPolygon()
  samplePoints.forEach(point => addPolygonPoint(point))
  
  // 设置起始点为多边形中心
  params.startPoint = {
    latitude: 39.8992,
    longitude: 116.4124
  }
}

const toggleWaypoints = (index: number) => {
  showWaypoints.value[index] = !showWaypoints.value[index]
}

// 格式化函数
const formatDistance = (distance: number): string => {
  if (distance >= 1000) {
    return `${(distance / 1000).toFixed(2)} 公里`
  }
  return `${distance.toFixed(1)} 米`
}

const formatTime = (minutes: number): string => {
  const hours = Math.floor(minutes / 60)
  const mins = Math.floor(minutes % 60)
  
  if (hours > 0) {
    return `${hours} 小时 ${mins} 分钟`
  }
  return `${mins} 分钟`
}

const formatArea = (area: number): string => {
  if (area >= 1000000) {
    return `${(area / 1000000).toFixed(2)} 平方公里`
  }
  return `${area.toFixed(0)} 平方米`
}

// 导出功能
const exportGeoJSON = () => {
  try {
    const geoJSON = exportToGeoJSON()
    const blob = new Blob([JSON.stringify(geoJSON, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `oblique_photography_routes_${new Date().toISOString().slice(0, 10)}.geojson`
    a.click()
    URL.revokeObjectURL(url)
  } catch (err) {
    console.error('导出GeoJSON失败:', err)
  }
}

const exportCSV = () => {
  if (!result.value) return
  
  const csvData = []
  csvData.push(['航线', '方向', '俯仰角', '航点序号', '纬度', '经度'])
  
  result.value.routes.forEach((route, routeIndex) => {
    route.waypoints.forEach((waypoint, waypointIndex) => {
      csvData.push([
        `第${routeIndex + 1}条航线`,
        route.direction.toFixed(1),
        route.gimbalPitch.toFixed(1),
        waypointIndex + 1,
        waypoint.latitude.toFixed(6),
        waypoint.longitude.toFixed(6)
      ])
    })
  })
  
  const csvContent = csvData.map(row => row.join(',')).join('\n')
  const blob = new Blob(['\ufeff' + csvContent], { type: 'text/csv;charset=utf-8' })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = `oblique_photography_waypoints_${new Date().toISOString().slice(0, 10)}.csv`
  a.click()
  URL.revokeObjectURL(url)
}

const exportSummary = () => {
  if (!result.value) return
  
  const summary = {
    规划时间: new Date().toLocaleString(),
    总航线数: result.value.totalRouteCount,
    总距离: formatDistance(result.value.totalDistance),
    预估作业时间: formatTime(estimatedWorkTime.value),
    边缘覆盖率: `${result.value.edgeCoverageRate.toFixed(1)}%`,
    是否优化: result.value.isOptimized ? '是' : '否',
    优化说明: result.value.optimizationReason,
    扩展距离: `${result.value.expansionDistance.toFixed(1)}米`,
    飞行参数: {
      主航线方向: `${params.mainDirection}°`,
      飞行高度: `${params.flightHeight}米`,
      云台俯仰角: `${params.gimbalPitch}°`,
      照片尺寸: `${params.photoWidth}×${params.photoLength}米`,
      旁路覆盖率: `${params.sideOverlapRate}%`,
      前向覆盖率: `${params.forwardOverlapRate}%`
    }
  }
  
  const blob = new Blob([JSON.stringify(summary, null, 2)], { type: 'application/json' })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = `oblique_photography_summary_${new Date().toISOString().slice(0, 10)}.json`
  a.click()
  URL.revokeObjectURL(url)
}
</script>

<style scoped>
.oblique-photography-planner {
  max-width: 1200px;
  margin: 0 auto;
  padding: 20px;
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
}

.planner-header {
  text-align: center;
  margin-bottom: 30px;
}

.title {
  font-size: 2.5rem;
  color: #2c3e50;
  margin-bottom: 10px;
}

.subtitle {
  font-size: 1.2rem;
  color: #7f8c8d;
  margin: 0;
}

.parameter-section, .results-section {
  background: white;
  border-radius: 12px;
  padding: 24px;
  margin-bottom: 24px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.parameter-section h2, .results-section h2 {
  color: #2c3e50;
  margin-bottom: 20px;
  border-bottom: 2px solid #3498db;
  padding-bottom: 10px;
}

.preset-section {
  margin-bottom: 24px;
}

.preset-buttons {
  display: flex;
  gap: 10px;
  flex-wrap: wrap;
  margin-top: 8px;
}

.preset-btn {
  padding: 8px 16px;
  border: 2px solid #3498db;
  background: white;
  color: #3498db;
  border-radius: 6px;
  cursor: pointer;
  transition: all 0.3s ease;
}

.preset-btn:hover {
  background: #3498db;
  color: white;
}

.parameter-group {
  margin-bottom: 24px;
  border: 1px solid #ecf0f1;
  border-radius: 8px;
  padding: 20px;
}

.parameter-group h3 {
  color: #34495e;
  margin-bottom: 16px;
}

.input-grid {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 16px;
}

.input-field {
  display: flex;
  flex-direction: column;
}

.input-field label {
  font-weight: 600;
  color: #2c3e50;
  margin-bottom: 6px;
}

.input-field input {
  padding: 10px;
  border: 2px solid #ecf0f1;
  border-radius: 6px;
  font-size: 14px;
  transition: border-color 0.3s ease;
}

.input-field input:focus {
  outline: none;
  border-color: #3498db;
}

.input-field small {
  color: #7f8c8d;
  font-size: 12px;
  margin-top: 4px;
}

.polygon-section {
  margin-top: 16px;
}

.polygon-header {
  display: flex;
  justify-content: space-between;
  align-items: center;
  margin-bottom: 12px;
}

.polygon-points {
  max-height: 300px;
  overflow-y: auto;
  border: 1px solid #ecf0f1;
  border-radius: 6px;
  padding: 12px;
}

.point-row {
  display: flex;
  align-items: center;
  gap: 10px;
  margin-bottom: 8px;
}

.point-index {
  font-weight: 600;
  color: #7f8c8d;
  min-width: 24px;
}

.point-row input {
  flex: 1;
  padding: 6px;
  border: 1px solid #ecf0f1;
  border-radius: 4px;
}

.polygon-actions {
  margin-top: 12px;
  display: flex;
  gap: 10px;
}

.action-section {
  display: flex;
  gap: 16px;
  margin-top: 24px;
}

.btn-primary, .btn-secondary, .btn-danger {
  padding: 12px 24px;
  border-radius: 6px;
  border: none;
  cursor: pointer;
  font-weight: 600;
  transition: all 0.3s ease;
}

.btn-primary {
  background: #3498db;
  color: white;
}

.btn-primary:hover:not(:disabled) {
  background: #2980b9;
}

.btn-primary:disabled {
  background: #bdc3c7;
  cursor: not-allowed;
}

.btn-secondary {
  background: #ecf0f1;
  color: #2c3e50;
}

.btn-secondary:hover {
  background: #d5dbdb;
}

.btn-danger {
  background: #e74c3c;
  color: white;
  padding: 4px 8px;
  font-size: 12px;
}

.btn-danger:hover {
  background: #c0392b;
}

.plan-btn {
  font-size: 16px;
  padding: 14px 32px;
}

.error-section {
  background: #fadbd8;
  border: 1px solid #e74c3c;
  border-radius: 6px;
  padding: 16px;
  margin-bottom: 24px;
}

.error-message {
  color: #c0392b;
}

.result-summary {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  margin-bottom: 30px;
}

.summary-card {
  background: #f8f9fa;
  border-radius: 8px;
  padding: 20px;
  border: 1px solid #ecf0f1;
}

.summary-card h3 {
  color: #2c3e50;
  margin-bottom: 16px;
}

.stats {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.stat {
  display: flex;
  justify-content: space-between;
  align-items: center;
}

.stat label {
  font-weight: 600;
  color: #34495e;
}

.stat span {
  color: #2c3e50;
  font-weight: 500;
}

.optimization-info {
  display: flex;
  flex-direction: column;
  gap: 12px;
}

.optimization-status .status {
  padding: 6px 12px;
  border-radius: 20px;
  font-size: 12px;
  font-weight: 600;
  background: #ecf0f1;
  color: #7f8c8d;
}

.optimization-status .status.optimized {
  background: #d5f4e6;
  color: #27ae60;
}

.optimization-reason {
  color: #7f8c8d;
  font-size: 14px;
  margin: 0;
  line-height: 1.5;
}

.route-details {
  margin-bottom: 30px;
}

.route-list {
  display: flex;
  flex-direction: column;
  gap: 16px;
}

.route-item {
  border: 1px solid #ecf0f1;
  border-radius: 8px;
  padding: 20px;
  background: #fafbfc;
}

.route-header {
  display: flex;
  justify-content: space-between;
  align-items: flex-start;
  margin-bottom: 12px;
}

.route-header h4 {
  color: #2c3e50;
  margin: 0;
}

.route-stats {
  display: flex;
  gap: 16px;
  font-size: 14px;
  color: #7f8c8d;
}

.waypoints-section {
  margin-top: 16px;
  border-top: 1px solid #ecf0f1;
  padding-top: 16px;
}

.waypoints-header {
  font-weight: 600;
  color: #34495e;
  margin-bottom: 12px;
}

.waypoints-list {
  max-height: 200px;
  overflow-y: auto;
  background: white;
  border-radius: 6px;
  padding: 12px;
}

.waypoint-item {
  display: flex;
  gap: 12px;
  padding: 4px 0;
  font-family: 'Courier New', monospace;
  font-size: 13px;
}

.waypoint-index {
  color: #7f8c8d;
  min-width: 24px;
}

.coordinates {
  color: #2c3e50;
}

.waypoints-toggle {
  margin-top: 12px;
  font-size: 14px;
  padding: 8px 16px;
}

.export-section {
  border-top: 1px solid #ecf0f1;
  padding-top: 20px;
}

.export-buttons {
  display: flex;
  gap: 12px;
  flex-wrap: wrap;
}

@media (max-width: 768px) {
  .oblique-photography-planner {
    padding: 10px;
  }
  
  .input-grid {
    grid-template-columns: 1fr;
  }
  
  .result-summary {
    grid-template-columns: 1fr;
  }
  
  .route-header {
    flex-direction: column;
    gap: 8px;
  }
  
  .route-stats {
    flex-direction: column;
    gap: 4px;
  }
  
  .export-buttons {
    flex-direction: column;
  }
}
</style>