import java.util.ArrayList;

public class DeterministicSolution {
    public double[] Y_value_IP;  //每种原材料进货量
    public double[][] Ic_value_IP;  //每周期末原材料库存量 [i][t]
    public double[][] Ie_value_IP;  //每周期末成品库存量 [k][t]
    public double[][][] Q_value_IP;  //每周期的切割方案使用量 [i][j][t]
    public double[][] B_value_IP;  //每周期末成品缺货量 [k][t]
    public double obj_value_IP;  //目标函数值
    
    public DeterministicSolution(ArrayList<Material> rawMaterials, ArrayList<Block> blocks, 
                                ArrayList<ArrayList<Pattern>> allPatterns, int T) {
        Y_value_IP = new double[rawMaterials.size()];
        Ic_value_IP = new double[rawMaterials.size()][T];
        Ie_value_IP = new double[blocks.size()][T + 1];
        B_value_IP = new double[blocks.size()][T + 1];
        Q_value_IP = new double[rawMaterials.size()][][];
        
        for(int i = 0; i < rawMaterials.size(); i++) {
            Q_value_IP[i] = new double[allPatterns.get(i).size()][T];
        }
    }
}






