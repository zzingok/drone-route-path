# 倾斜摄影5航线算法 - Vue.js 版本

这是从 Java 版本转换而来的 Vue.js 倾斜摄影航线规划系统，用于智能无人机航线规划。

## 功能特性

### 🚁 智能航线规划
- **自适应航线数量**: 根据云台俯仰角自动判断所需航线数量（1-5条）
- **优化算法**: 智能优化航线数量，提高作业效率
- **边缘覆盖保证**: 自动扩展作业区域，确保边缘完全覆盖

### 📊 灵活参数配置
- **飞行参数**: 主航线方向、飞行高度、云台俯仰角
- **拍摄参数**: 照片尺寸、重叠率设置
- **作业区域**: 支持自定义多边形区域和起始点
- **预设配置**: 提供多种预设方案（低空精细、中等标准、高空大范围、超高精度）

### 🎯 精确计算
- **距离计算**: 使用Haversine公式精确计算地球表面距离
- **覆盖率分析**: 实时计算边缘覆盖率和区域扩展信息
- **作业时间估算**: 根据航线距离和拍摄参数估算作业时间

### 📤 多格式导出
- **GeoJSON**: 完整的地理信息数据
- **CSV**: 航点坐标数据
- **JSON**: 规划摘要信息

## 项目结构

```
├── types/
│   └── drone-route.ts              # TypeScript 类型定义
├── utils/
│   ├── drone-algorithm.ts          # 核心算法工具函数
│   └── oblique-photography-algorithm.ts  # 主要算法实现
├── composables/
│   └── useObliquePhotography.ts    # Vue 组合式API
├── components/
│   └── ObliquePhotographyPlanner.vue    # 主要UI组件
└── examples/
    └── basic-usage.vue             # 使用示例
```

## 快速开始

### 1. 安装依赖

确保你的项目支持 TypeScript 和 Vue 3。

### 2. 导入组件

```vue
<template>
  <div>
    <ObliquePhotographyPlanner />
  </div>
</template>

<script setup>
import ObliquePhotographyPlanner from '~/components/ObliquePhotographyPlanner.vue'
</script>
```

### 3. 使用 Composable API

```typescript
import { useObliquePhotography } from '~/composables/useObliquePhotography'

const {
  params,
  result,
  isLoading,
  error,
  planRoutes,
  setPolygon,
  exportToGeoJSON
} = useObliquePhotography()

// 设置作业区域
setPolygon([
  { latitude: 39.9042, longitude: 116.4074 },
  { latitude: 39.9042, longitude: 116.4174 },
  { latitude: 39.8942, longitude: 116.4174 },
  { latitude: 39.8942, longitude: 116.4074 }
])

// 规划航线
await planRoutes({
  mainDirection: 0,
  flightHeight: 100,
  gimbalPitch: -30,
  sideOverlapRate: 80,
  forwardOverlapRate: 80,
  photoWidth: 50,
  photoLength: 50
})

// 导出结果
const geoJSON = exportToGeoJSON()
```

## API 文档

### 类型定义

#### LatLngPoint
```typescript
interface LatLngPoint {
  latitude: number;   // 纬度
  longitude: number;  // 经度
  id?: number;        // 可选ID
}
```

#### ObliqueParams
```typescript
interface ObliqueParams {
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
```

#### ObliqueRouteResult
```typescript
interface ObliqueRouteResult {
  routes: RouteDirection[];       // 各方向航线列表
  totalDistance: number;          // 总距离（米）
  totalRouteCount: number;        // 实际航线数量
  isOptimized: boolean;          // 是否为优化后的航线
  optimizationReason: string;    // 优化原因说明
  expandedPolygon: LatLngPoint[]; // 扩展后的作业区域
  expansionDistance: number;      // 扩展距离（米）
  edgeCoverageRate: number;      // 边缘覆盖率估算
}
```

### 主要方法

#### planObliqueRoutes(params: ObliqueParams)
规划倾斜摄影航线的主要方法。

**参数:**
- `params`: 倾斜摄影参数配置

**返回值:**
- `Promise<ObliqueRouteResult>`: 规划结果

#### getRecommendedGimbalPitch(targetHeight: number, cameraFOV: number)
根据飞行高度获取推荐的云台俯仰角。

**参数:**
- `targetHeight`: 目标飞行高度（米）
- `cameraFOV`: 相机视场角（度）

**返回值:**
- `number`: 推荐的云台俯仰角（度）

### Composable API

#### useObliquePhotography()
主要的组合式API，提供完整的航线规划功能。

**返回值:**
```typescript
{
  // 状态
  isLoading: Ref<boolean>
  error: Ref<string | null>
  result: Ref<ObliqueRouteResult | null>
  expandedAreaInfo: Ref<ExpandedAreaInfo | null>
  
  // 参数
  params: ObliqueParams
  
  // 计算属性
  hasValidPolygon: ComputedRef<boolean>
  estimatedWorkTime: ComputedRef<number>
  recommendedGimbalPitch: ComputedRef<number>
  
  // 方法
  planRoutes: (params?: Partial<ObliqueParams>) => Promise<ObliqueRouteResult>
  updateParams: (params: Partial<ObliqueParams>) => void
  setPolygon: (points: LatLngPoint[]) => void
  addPolygonPoint: (point: LatLngPoint) => void
  removePolygonPoint: (index: number) => void
  clearPolygon: () => void
  setStartPoint: (point: LatLngPoint) => void
  resetParams: () => void
  exportToGeoJSON: () => object
  importFromGeoJSON: (geoJSON: any) => void
}
```

## 预设配置

系统提供了4种预设配置：

### 低空精细摄影 (lowAltitudeDetailed)
- 飞行高度: 50米
- 云台俯仰角: -20°
- 重叠率: 85%
- 照片尺寸: 30×30米

### 中等标准摄影 (mediumAltitudeStandard)
- 飞行高度: 100米
- 云台俯仰角: -30°
- 重叠率: 80%
- 照片尺寸: 50×50米

### 高空大范围摄影 (highAltitudeWide)
- 飞行高度: 200米
- 云台俯仰角: -45°
- 重叠率: 75%
- 照片尺寸: 100×100米

### 超高精度摄影 (ultraHighPrecision)
- 飞行高度: 30米
- 云台俯仰角: -15°
- 重叠率: 90%
- 照片尺寸: 20×20米

## 算法特性

### 智能优化策略

系统根据云台俯仰角自动选择最优的航线数量：

1. **角度 < 15°**: 使用单方向垂直摄影
2. **15° ≤ 角度 < 30°**: 使用3方向十字交叉摄影
3. **30° ≤ 角度 < 45°**: 使用4方向正交摄影
4. **角度 ≥ 45°**: 使用完整5方向摄影

### 边缘覆盖保证

- 自动计算扩展距离
- 确保边缘区域完全覆盖
- 实时计算覆盖率

### 性能优化

- 使用缓存机制减少重复计算
- 预计算三角函数表
- 空间索引优化邻近查询
- 对象池减少内存分配

## 浏览器兼容性

- Chrome 90+
- Firefox 88+
- Safari 14+
- Edge 90+

## 许可证

MIT License

## 从 Java 版本的主要变化

1. **类型系统**: 使用 TypeScript 接口替代 Java 类
2. **异步处理**: 使用 Promise/async-await 替代同步方法
3. **响应式**: 集成 Vue 3 响应式系统
4. **模块化**: 采用 ES6 模块系统
5. **函数式**: 优先使用函数式编程范式
6. **性能**: 利用 JavaScript 特性进行性能优化

## 贡献指南

欢迎提交 Issue 和 Pull Request 来改进这个项目。

## 支持

如果你在使用过程中遇到问题，请创建 Issue 或联系维护者。