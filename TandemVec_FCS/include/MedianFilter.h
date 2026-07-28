// MedianFilter.h
#ifndef MEDIAN_FILTER_H
#define MEDIAN_FILTER_H

/**
 * @brief 优化版中值滤波器 - 采用快速选择算法加速中值计算，支持浮点数。
 * @note 窗口大小建议范围3-15（奇数），窗口越大，滤波效果越好，但计算复杂度也越高。
 *       在STM32H7等带硬件浮点单元(FPU)的芯片上，浮点数运算性能很高。
 * @version 2.2 (修改为支持float类型数据)
 */
class MedianFilter
{
private:
    const int windowSize; // 滤波窗口的大小，必须为奇数，建议3/5/7/9等小尺寸
    float *window;        // 循环缓冲区，用于存储最新windowSize个数据点，使用float类型
    int windowPos;        // 循环缓冲区的当前写入位置（0到windowSize-1）
    bool isInitialized;   // 窗口是否已完全填满至少一次的标志
    float *tempArray;     // 用于中值计算的临时数组，复制window数据到此处进行操作，避免影响原始window，使用float类型

    /**
     * @brief 快速选择算法中的分区函数（Partition）。
     *        选择一个基准元素（这里选择最右边的元素），将数组分为两部分：
     *        小于等于基准的元素移到左边，大于基准的元素移到右边。
     * @param arr 待分区的浮点数数组
     * @param left 分区范围的左边界索引
     * @param right 分区范围的右边界索引
     * @return 基准元素最终所在位置的索引
     */
    int partition(float arr[], int left, int right)
    {
        float pivot = arr[right]; // 选择最右元素作为基准（pivot）
        int i = left - 1;         // 初始化一个指针i，指向小于基准的区域的边界

        // 遍历当前分区范围内的元素
        for (int j = left; j < right; j++)
        {
            // 如果当前元素小于或等于基准
            if (arr[j] <= pivot)
            {
                i++; // 扩展小于基准的区域边界
                // 交换arr[i]和arr[j]，将小于等于基准的元素移到左边
                float temp = arr[i];
                arr[i] = arr[j];
                arr[j] = temp;
            }
        }

        // 循环结束后，i+1的位置就是基准元素应放置的最终位置
        // 将基准元素（arr[right]）放到正确的位置上
        float temp = arr[i + 1];
        arr[i + 1] = arr[right];
        arr[right] = temp;

        return i + 1; // 返回基准元素的最终索引
    }

    /**
     * @brief 快速选择算法实现（QuickSelect）。
     *        用于在平均线性时间复杂度O(N)内找到数组中第k小的元素。
     *        这里用于查找中值，即第 (windowSize / 2) 小的元素。
     * @param arr 待处理的浮点数数组（已从window复制过来）
     * @param left 当前查找范围的左边界索引
     * @param right 当前查找范围的右边界索引
     * @param k 目标元素在 *整个数组* 中的索引（从0开始计数），这里是中值索引
     * @return 第k小的浮点数元素值（即中值）
     */
    float quickSelect(float arr[], int left, int right, int k)
    {
        // 使用循环替代递归，避免栈溢出，适合嵌入式系统
        while (left <= right)
        {
            // 对当前范围[left, right]进行分区
            int pivotIndex = partition(arr, left, right);

            // 检查基准元素的位置pivotIndex
            if (pivotIndex == k)
            {
                // 如果基准元素恰好是我们寻找的第k小元素，直接返回其值
                return arr[pivotIndex];
            }
            else if (pivotIndex < k)
            {
                // 如果基准元素位置在k的左边，说明第k小元素在基准元素的右侧
                // 在基准的右侧子数组 [pivotIndex + 1, right] 中继续查找
                left = pivotIndex + 1;
            }
            else
            {
                // 如果基准元素位置在k的右边，说明第k小元素在基准元素的左侧
                // 在基准的左侧子数组 [left, pivotIndex - 1] 中继续查找
                right = pivotIndex - 1;
            }
        }

        // 如果查找失败（理论上对于有效的k和数组不会走到这里），
        // 或者left > right时（循环终止），返回left位置的值。
        // 对于中值查找，k总是有效的，通常不会依赖此处的返回值。
        // 这是一个理论上的回退，实际应用中QuickSelect会找到k。
        return arr[left];
    }

public:
    /**
     * @brief 构造函数。创建中值滤波器实例。
     *        自动调整窗口大小为不小于3的奇数。
     * @param size 期望的滤波窗口大小。
     */
    explicit MedianFilter(int size)
        // 初始化成员变量：
        // 确保windowSize至少为3，且为奇数。如果输入是偶数，加1；如果小于3，设为3。
        : windowSize((size < 3) ? 3 : (size % 2 == 0 ? size + 1 : size))
    {
        // 在堆上分配循环缓冲区和临时数组的内存
        // 使用new[]分配float类型数组
        window = new float[windowSize];
        tempArray = new float[windowSize]; // 预分配排序/选择的临时空间

        // 检查内存分配是否成功（在某些Arduino/嵌入式环境下new可能返回nullptr）
        // 注意：标准的C++ new失败会抛异常，但很多嵌入式实现不会。
        // 稳健的做法应检查window和tempArray是否为nullptr。
        // 此处简化处理，假设分配成功，实际应用中考虑加入错误处理。

        windowPos = 0;         // 初始化写入位置为0
        isInitialized = false; // 初始时窗口未填满

        // 初始化窗口内的所有元素为0.0f
        for (int i = 0; i < windowSize; i++)
        {
            window[i] = 0.0f; // 使用浮点数0初始化
        }
    }

    /**
     * @brief 析构函数。释放动态分配的内存。
     *        在对象生命周期结束时自动调用，防止内存泄漏。
     */
    ~MedianFilter()
    {
        // 释放window和tempArray占用的内存
        delete[] window;
        delete[] tempArray;
        // 将指针设为nullptr是一种好的习惯，避免野指针，尽管析构后对象本身就销毁了
        window = nullptr;
        tempArray = nullptr;
    }

    /**
     * @brief 实时滤波处理函数。
     *        接收新的传感器数据，更新滑动窗口，并在窗口填满后计算中值。
     * @param currentValue 新采集的原始浮点数数据
     * @return 滤波后的浮点数数据。在窗口未填满时，返回原始值。
     */
    float filter(float currentValue)
    {
        // 将新的数据写入循环缓冲区当前位置
        window[windowPos] = currentValue;

        // 更新写入位置，使用模运算实现循环
        windowPos = (windowPos + 1) % windowSize;

        // 检查窗口是否已首次填满
        if (!isInitialized)
        {
            // 窗口填满的条件是windowPos第一次回绕到0
            if (windowPos == 0)
            {
                isInitialized = true; // 标记窗口已初始化
            }
            // 在窗口未填满之前，返回原始数据，不进行滤波
            return currentValue;
        }

        // 窗口已填满，开始计算中值

        // 将循环缓冲区中的当前数据按顺序复制到临时数组
        // tempArray用于进行排序/选择操作，避免修改原始window数据
        // 从windowPos开始复制，windowPos是下一个待写入位置，所以当前窗口数据从其前面开始
        for (int i = 0; i < windowSize; i++)
        {
            // 通过(windowPos + i) % windowSize 计算出window中元素的正确索引
            // 这样复制到tempArray中的数据是按时间顺序排列的（逻辑上的顺序，尽管在window中是循环的）
             tempArray[i] = window[(windowPos + i) % windowSize];
        }

        // 使用快速选择算法计算中值
        // 中值是排序后的第 (windowSize / 2) 小的元素
        // QuickSelect函数的k参数是索引，所以是 windowSize / 2
        return quickSelect(
            tempArray,          // 在复制出的临时数组上操作
            0,                  // 查找范围左边界索引
            windowSize - 1,     // 查找范围右边界索引
            windowSize / 2      // 目标元素的索引 (对于奇数大小的窗口，这就是中间元素的索引)
        );
    }

    /**
     * @brief 快速初始化窗口。
     *        在启动阶段调用此函数，可以用一个指定值填充整个窗口，
     *        从而避免启动阶段因窗口未填满而直接输出原始值的行为，使滤波效果即时生效。
     * @param initValue 用于填充窗口的浮点数初始化值。
     */
    void initialize(float initValue)
    {
        // 用initValue填充整个循环缓冲区
        for (int i = 0; i < windowSize; i++)
        {
            window[i] = initValue;
        }
        isInitialized = true; // 标记窗口已初始化
        windowPos = 0; // 重置写入位置，下次filter调用会从头开始写
    }

    /**
     * @brief 获取当前滤波器配置的窗口大小。
     * @return 实际生效的窗口大小（一个不小于3的奇数）。
     */
    int getWindowSize() const
    {
        return windowSize;
    }
};

#endif // MEDIAN_FILTER_H