public class Parameters {
    public static int nodeNumPerLayer = 10;  //场景树每层的节点数
    public static int leadTime = 0;
    public static int maxIter = 80;  //反向传播迭代轮数
    public static int simulatePaths = 100; //模拟评估路径数
    public static double cutCost = 1;  //切割成本
    public static int timeLimit = 120;
    // 需求生成相关参数
    public static String demandSampling = "normal"; // 可选: "poisson", "normal"
    public static double demandStdFactor = 15.0; // 正态采样时的标准差因子（乘以 sqrt(mu)）
}
