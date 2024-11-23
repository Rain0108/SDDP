import java.util.ArrayList;
import java.util.HashMap;

public class Solution {
    public ArrayList<Integer> path;  //当前解对应的场景路径
    public double[] Y_value;  //每种原材料进货量
    public double[][][] Ic_value;  //每周期末原材料库存量
    public double[][][] Ie_value;  //每周期末成品库存量
    public double[][][][] Q_value;  //每周期的切割方案使用量   [0,T-1]
    public double[][][] B_value;  //每周期末成品缺货量  [1,T]
    public double[][] obj_value;  //每个周期的目标函数值  [0,T]
    public double[][] F_value;
    public double[] Y_value_IP;  //每种原材料进货量
    public double[][][] Ic_value_IP;  //每周期末原材料库存量
    public double[][][] Ie_value_IP;  //每周期末成品库存量
    public double[][][][] Q_value_IP;  //每周期的切割方案使用量   [0,T-1]
    public double[][][] B_value_IP;  //每周期末成品缺货量  [1,T]
    public double[][] obj_value_IP;  //每个周期的目标函数值  [0,T]
    public double[][] F_value_IP;
    public Solution(ArrayList<Material> rawMaterials, ArrayList<Block> blocks, ArrayList<ArrayList<Pattern>> allPatterns,
                    int T, ArrayList<Integer> path) {
        Y_value = new double[rawMaterials.size()];
        Ic_value = new double[Parameters.nodeNumPerLayer][][];
        Ie_value = new double[Parameters.nodeNumPerLayer][][];
        B_value = new double[Parameters.nodeNumPerLayer][][];
        obj_value = new double[Parameters.nodeNumPerLayer][];
        F_value = new double[Parameters.nodeNumPerLayer][];
        Q_value = new double[Parameters.nodeNumPerLayer][][][];
        Y_value_IP = new double[rawMaterials.size()];
        Ic_value_IP = new double[Parameters.nodeNumPerLayer][][];
        Ie_value_IP = new double[Parameters.nodeNumPerLayer][][];
        B_value_IP = new double[Parameters.nodeNumPerLayer][][];
        obj_value_IP = new double[Parameters.nodeNumPerLayer][];
        F_value_IP = new double[Parameters.nodeNumPerLayer][];
        Q_value_IP = new double[Parameters.nodeNumPerLayer][][][];
        for (int n=0;n<Parameters.nodeNumPerLayer;n++){
            Q_value[n] = new double[rawMaterials.size()][][];
            obj_value[n] = new double[T+1];
            F_value[n] = new double[T];
            B_value[n] = new double[blocks.size()][];
            Ie_value[n] = new double[blocks.size()][];
            Ic_value[n] = new double[rawMaterials.size()][];
            Q_value_IP[n] = new double[rawMaterials.size()][][];
            obj_value_IP[n] = new double[T+1];
            F_value_IP[n] = new double[T];
            B_value_IP[n] = new double[blocks.size()][];
            Ie_value_IP[n] = new double[blocks.size()][];
            Ic_value_IP[n] = new double[rawMaterials.size()][];
            for(int i=0;i< rawMaterials.size();i++){
                Ic_value[n][i] = new double[T];
                Q_value[n][i] = new double[allPatterns.get(i).size()][];
                Ic_value_IP[n][i] = new double[T];
                Q_value_IP[n][i] = new double[allPatterns.get(i).size()][];
                for (int j = 0; j < allPatterns.get(i).size(); j++) {
                    Q_value[n][i][j] = new double[T];
                    Q_value_IP[n][i][j] = new double[T];
                }
            }
            for(int i=0;i< blocks.size();i++){
                Ie_value[n][i] = new double[T+1];
                B_value[n][i] = new double[T+1];
                Ie_value_IP[n][i] = new double[T+1];
                B_value_IP[n][i] = new double[T+1];
            }
        }
        this.path = path;
    }
}
