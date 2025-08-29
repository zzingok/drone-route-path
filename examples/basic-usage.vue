<template>
  <div class="example-container">
    <h1>倾斜摄影航线规划 - 基本使用示例</h1>
    
    <!-- 简单示例 -->
    <section class="example-section">
      <h2>1. 基本使用</h2>
      <div class="example-content">
        <div class="example-controls">
          <button @click="runBasicExample" :disabled="isLoading" class="btn-primary">
            {{ isLoading ? '规划中...' : '运行基本示例' }}
          </button>
          <button @click="resetExample" class="btn-secondary">
            重置
          </button>
        </div>
        
        <div v-if="error" class="error-display">
          <strong>错误:</strong> {{ error }}
        </div>
        
        <div v-if="result" class="result-display">
          <h3>规划结果</h3>
          <div class="result-summary">
            <p><strong>航线数量:</strong> {{ result.totalRouteCount }} 条</p>
            <p><strong>总距离:</strong> {{ formatDistance(result.totalDistance) }}</p>
            <p><strong>是否优化:</strong> {{ result.isOptimized ? '是' : '否' }}</p>
            <p><strong>优化说明:</strong> {{ result.optimizationReason }}</p>
            <p><strong>边缘覆盖率:</strong> {{ result.edgeCoverageRate.toFixed(1) }}%</p>
          </div>
        </div>
      </div>
    </section>

    <!-- 预设配置示例 -->
    <section class="example-section">
      <h2>2. 预设配置示例</h2>
      <div class="example-content">
        <div class="preset-examples">
          <div 
            v-for="(config, key) in presetConfigs" 
            :key="key"
            class="preset-card"
          >
            <h4>{{ getPresetName(key) }}</h4>
            <div class="preset-details">
              <p>飞行高度: {{ config.flightHeight }}米</p>
              <p>云台角度: {{ config.gimbalPitch }}°</p>
              <p>重叠率: {{ config.sideOverlapRate }}%</p>
            </div>
            <button 
              @click="runPresetExample(key)" 
              :disabled="isLoading"
              class="btn-secondary"
            >
              运行此配置
            </button>
          </div>
        </div>
      </div>
    </section>

    <!-- 编程式API示例 -->
    <section class="example-section">
      <h2>3. 编程式 API 使用</h2>
      <div class="example-content">
        <pre class="code-example"><code>{{ apiExampleCode }}</code></pre>
        <button @click="runApiExample" :disabled="isLoading" class="btn-primary">
          运行 API 示例
        </button>
      </div>
    </section>

    <!-- 数据导出示例 -->
    <section class="example-section">
      <h2>4. 数据导出示例</h2>
      <div class="example-content">
        <div class="export-controls">
          <button 
            @click="exportExampleGeoJSON" 
            :disabled="!result"
            class="btn-secondary"
          >
            导出 GeoJSON
          </button>
          <button 
            @click="exportExampleCSV" 
            :disabled="!result"
            class="btn-secondary"
          >
            导出 CSV
          </button>
          <button 
            @click="showGeoJSONPreview" 
            :disabled="!result"
            class="btn-secondary"
          >
            预览 GeoJSON
          </button>
        </div>
        
        <div v-if="geoJSONPreview" class="json-preview">
          <h4>GeoJSON 预览:</h4>
          <pre><code>{{ JSON.stringify(geoJSONPreview, null, 2).slice(0, 1000) }}...</code></pre>
        </div>
      </div>
    </section>

    <!-- 实时参数调整示例 -->
    <section class="example-section">
      <h2>5. 实时参数调整</h2>
      <div class="example-content">
        <div class="parameter-controls">
          <div class="control-group">
            <label>飞行高度 ({{ params.flightHeight }}米)</label>
            <input 
              v-model.number="params.flightHeight" 
              type="range" 
              min="30" 
              max="300" 
              step="10"
              @change="onParameterChange"
            />
          </div>
          
          <div class="control-group">
            <label>云台俯仰角 ({{ params.gimbalPitch }}°)</label>
            <input 
              v-model.number="params.gimbalPitch" 
              type="range" 
              min="-90" 
              max="-5" 
              step="5"
              @change="onParameterChange"
            />
          </div>
          
          <div class="control-group">
            <label>旁路覆盖率 ({{ params.sideOverlapRate }}%)</label>
            <input 
              v-model.number="params.sideOverlapRate" 
              type="range" 
              min="50" 
              max="95" 
              step="5"
              @change="onParameterChange"
            />
          </div>
        </div>
        
        <div class="real-time-info">
          <p><strong>推荐云台角度:</strong> {{ recommendedGimbalPitch }}°</p>
          <p><strong>预估作业时间:</strong> {{ formatTime(estimatedWorkTime) }}</p>
        </div>
      </div>
    </section>
  </div>
</template>

<script setup lang="ts">
import { ref, computed, watch } from 'vue'
import { useObliquePhotography, presetConfigs } from '~/composables/useObliquePhotography'

// 使用组合式API
const {
  isLoading,
  error,
  result,
  params,
  estimatedWorkTime,
  recommendedGimbalPitch,
  planRoutes,
  updateParams,
  setPolygon,
  setStartPoint,
  resetParams,
  exportToGeoJSON
} = useObliquePhotography()

// 示例状态
const geoJSONPreview = ref(null)

// 示例数据
const samplePolygon = [
  { latitude: 39.9042, longitude: 116.4074 },
  { latitude: 39.9142, longitude: 116.4074 },
  { latitude: 39.9142, longitude: 116.4174 },
  { latitude: 39.9042, longitude: 116.4174 }
]

const sampleStartPoint = { latitude: 39.9092, longitude: 116.4124 }

// API 示例代码
const apiExampleCode = `// 1. 导入组合式API
import { useObliquePhotography } from '~/composables/useObliquePhotography'

// 2. 初始化
const {
  planRoutes,
  setPolygon,
  setStartPoint,
  result,
  isLoading,
  error
} = useObliquePhotography()

// 3. 设置作业区域
setPolygon([
  { latitude: 39.9042, longitude: 116.4074 },
  { latitude: 39.9142, longitude: 116.4074 },
  { latitude: 39.9142, longitude: 116.4174 },
  { latitude: 39.9042, longitude: 116.4174 }
])

// 4. 设置起始点
setStartPoint({ latitude: 39.9092, longitude: 116.4124 })

// 5. 规划航线
await planRoutes({
  mainDirection: 45,
  flightHeight: 120,
  gimbalPitch: -35,
  sideOverlapRate: 80,
  forwardOverlapRate: 80,
  photoWidth: 60,
  photoLength: 60
})

// 6. 获取结果
console.log('规划结果:', result.value)`

// 方法
const runBasicExample = async () => {
  try {
    // 设置示例区域
    setPolygon(samplePolygon)
    setStartPoint(sampleStartPoint)
    
    // 使用基本参数规划航线
    await planRoutes({
      mainDirection: 0,
      flightHeight: 100,
      gimbalPitch: -30,
      sideOverlapRate: 80,
      forwardOverlapRate: 80,
      photoWidth: 50,
      photoLength: 50
    })
    
    console.log('基本示例完成:', result.value)
  } catch (err) {
    console.error('基本示例失败:', err)
  }
}

const resetExample = () => {
  resetParams()
  geoJSONPreview.value = null
}

const runPresetExample = async (presetKey: string) => {
  try {
    const config = presetConfigs[presetKey as keyof typeof presetConfigs]
    
    // 设置示例区域
    setPolygon(samplePolygon)
    setStartPoint(sampleStartPoint)
    
    // 应用预设配置
    await planRoutes({
      ...config,
      mainDirection: 45 // 设置主航线方向为45度
    })
    
    console.log(`预设示例 ${presetKey} 完成:`, result.value)
  } catch (err) {
    console.error(`预设示例 ${presetKey} 失败:`, err)
  }
}

const runApiExample = async () => {
  try {
    // 设置示例区域
    setPolygon(samplePolygon)
    setStartPoint(sampleStartPoint)
    
    // 使用API示例中的参数
    await planRoutes({
      mainDirection: 45,
      flightHeight: 120,
      gimbalPitch: -35,
      sideOverlapRate: 80,
      forwardOverlapRate: 80,
      photoWidth: 60,
      photoLength: 60
    })
    
    console.log('API示例完成:', result.value)
  } catch (err) {
    console.error('API示例失败:', err)
  }
}

const exportExampleGeoJSON = () => {
  try {
    const geoJSON = exportToGeoJSON()
    const blob = new Blob([JSON.stringify(geoJSON, null, 2)], { type: 'application/json' })
    const url = URL.createObjectURL(blob)
    const a = document.createElement('a')
    a.href = url
    a.download = `example_oblique_routes_${new Date().toISOString().slice(0, 10)}.geojson`
    a.click()
    URL.revokeObjectURL(url)
  } catch (err) {
    console.error('导出GeoJSON失败:', err)
  }
}

const exportExampleCSV = () => {
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
  a.download = `example_waypoints_${new Date().toISOString().slice(0, 10)}.csv`
  a.click()
  URL.revokeObjectURL(url)
}

const showGeoJSONPreview = () => {
  try {
    geoJSONPreview.value = exportToGeoJSON()
  } catch (err) {
    console.error('生成GeoJSON预览失败:', err)
  }
}

const onParameterChange = () => {
  // 可以在这里添加实时规划逻辑
  console.log('参数已更改:', {
    flightHeight: params.flightHeight,
    gimbalPitch: params.gimbalPitch,
    sideOverlapRate: params.sideOverlapRate
  })
}

// 工具函数
const getPresetName = (key: string): string => {
  const names: Record<string, string> = {
    lowAltitudeDetailed: '低空精细摄影',
    mediumAltitudeStandard: '中等标准摄影',
    highAltitudeWide: '高空大范围摄影',
    ultraHighPrecision: '超高精度摄影'
  }
  return names[key] || key
}

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

// 监听器
watch(() => params.flightHeight, (newHeight) => {
  console.log('飞行高度变更为:', newHeight)
})

watch(() => params.gimbalPitch, (newPitch) => {
  console.log('云台俯仰角变更为:', newPitch)
})
</script>

<style scoped>
.example-container {
  max-width: 1200px;
  margin: 0 auto;
  padding: 20px;
  font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
}

.example-section {
  background: white;
  border-radius: 12px;
  padding: 24px;
  margin-bottom: 24px;
  box-shadow: 0 4px 6px rgba(0, 0, 0, 0.1);
}

.example-section h2 {
  color: #2c3e50;
  margin-bottom: 20px;
  border-bottom: 2px solid #3498db;
  padding-bottom: 10px;
}

.example-content {
  margin-top: 16px;
}

.example-controls {
  display: flex;
  gap: 12px;
  margin-bottom: 20px;
}

.btn-primary, .btn-secondary {
  padding: 10px 20px;
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

.btn-secondary:hover:not(:disabled) {
  background: #d5dbdb;
}

.btn-secondary:disabled {
  background: #f8f9fa;
  color: #adb5bd;
  cursor: not-allowed;
}

.error-display {
  background: #fadbd8;
  border: 1px solid #e74c3c;
  border-radius: 6px;
  padding: 12px;
  margin: 16px 0;
  color: #c0392b;
}

.result-display {
  background: #d5f4e6;
  border: 1px solid #27ae60;
  border-radius: 6px;
  padding: 16px;
  margin: 16px 0;
}

.result-display h3 {
  color: #27ae60;
  margin-bottom: 12px;
}

.result-summary {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.result-summary p {
  margin: 0;
  color: #2c3e50;
}

.preset-examples {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
  gap: 16px;
}

.preset-card {
  border: 1px solid #ecf0f1;
  border-radius: 8px;
  padding: 16px;
  background: #fafbfc;
}

.preset-card h4 {
  color: #2c3e50;
  margin-bottom: 12px;
}

.preset-details {
  margin-bottom: 16px;
}

.preset-details p {
  margin: 4px 0;
  color: #7f8c8d;
  font-size: 14px;
}

.code-example {
  background: #2c3e50;
  color: #ecf0f1;
  padding: 20px;
  border-radius: 8px;
  overflow-x: auto;
  margin-bottom: 16px;
  font-family: 'Courier New', monospace;
  font-size: 14px;
  line-height: 1.5;
}

.export-controls {
  display: flex;
  gap: 12px;
  margin-bottom: 20px;
  flex-wrap: wrap;
}

.json-preview {
  background: #f8f9fa;
  border: 1px solid #dee2e6;
  border-radius: 6px;
  padding: 16px;
  margin-top: 16px;
}

.json-preview h4 {
  color: #495057;
  margin-bottom: 12px;
}

.json-preview pre {
  background: #2c3e50;
  color: #ecf0f1;
  padding: 16px;
  border-radius: 4px;
  overflow-x: auto;
  font-family: 'Courier New', monospace;
  font-size: 12px;
  line-height: 1.4;
  margin: 0;
}

.parameter-controls {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
  gap: 20px;
  margin-bottom: 20px;
}

.control-group {
  display: flex;
  flex-direction: column;
  gap: 8px;
}

.control-group label {
  font-weight: 600;
  color: #2c3e50;
}

.control-group input[type="range"] {
  width: 100%;
  height: 6px;
  background: #ecf0f1;
  border-radius: 3px;
  outline: none;
  cursor: pointer;
}

.control-group input[type="range"]::-webkit-slider-thumb {
  appearance: none;
  width: 20px;
  height: 20px;
  background: #3498db;
  border-radius: 50%;
  cursor: pointer;
}

.control-group input[type="range"]::-moz-range-thumb {
  width: 20px;
  height: 20px;
  background: #3498db;
  border-radius: 50%;
  cursor: pointer;
  border: none;
}

.real-time-info {
  background: #e8f4fd;
  border: 1px solid #3498db;
  border-radius: 6px;
  padding: 16px;
}

.real-time-info p {
  margin: 8px 0;
  color: #2c3e50;
  font-weight: 500;
}

@media (max-width: 768px) {
  .example-container {
    padding: 10px;
  }
  
  .preset-examples {
    grid-template-columns: 1fr;
  }
  
  .parameter-controls {
    grid-template-columns: 1fr;
  }
  
  .example-controls,
  .export-controls {
    flex-direction: column;
  }
}
</style>