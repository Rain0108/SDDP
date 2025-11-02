import java.util.ArrayList;

/**
 * NewMILPSolver的求解结果类
 * 结构与ExpectedDemandModel的DeterministicSolution保持一致
 */
public class NewMILPSolution {
    public double obj_value;           // 目标函数值
    public double[] Y_value;           // 第一期采购量 [i]
    public double[][][] Ic_value;      // 原材料库存 [i][s][t]
    public double[][][] Ie_value;      // 成品库存 [k][s][t]
    public double[][][] B_value;       // 成品缺货 [k][s][t]
    public double[][][][] Q_value;     // 切割方案 [i][j][s][t]
    
    public NewMILPSolution(ArrayList<Material> rawMaterials, ArrayList<Block> blocks, 
                          ArrayList<ArrayList<Pattern>> allPatterns, int numScenarios, int T) {
        Y_value = new double[rawMaterials.size()];
        Ic_value = new double[rawMaterials.size()][numScenarios][T];
        Ie_value = new double[blocks.size()][numScenarios][T + 1];
        B_value = new double[blocks.size()][numScenarios][T + 1];
        Q_value = new double[rawMaterials.size()][][][];
        
        for (int i = 0; i < rawMaterials.size(); i++) {
            Q_value[i] = new double[allPatterns.get(i).size()][numScenarios][T];
        }
    }
}

