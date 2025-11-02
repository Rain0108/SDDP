import ilog.concert.*;
import ilog.cplex.IloCplex;

import java.util.ArrayList;
import java.util.HashMap;

public class ExpectedDemandModel {
    public ArrayList<Material> rawMaterials;  //原材料集合
    public ArrayList<Block> blocks;  //零件集合
    public ArrayList<ArrayList<Pattern>> allPatterns;  //所有切割方案
    public int[][] D_it;  //成品i在t期的期望需求量
    public int T; //总周期数

    public IloCplex modelLP;
    public IloCplex modelIP;

    public IloNumVar[] Y_i;
    public IloNumVar[][] Ic_it;   // 原料库存 [i][t]
    public IloNumVar[][] Ie_it;   // 成品库存 [k][t] (t=0..T)
    public IloNumVar[][] B_it;    // 成品缺货 [k][t] (t=0..T)
    public IloNumVar[][][] Q_ijt; // 切割方案 [i][j][t]
    public IloIntVar[] Y_i_IP;
    public IloIntVar[][] Ic_it_IP;
    public IloIntVar[][] Ie_it_IP;
    public IloIntVar[][] B_it_IP;
    public IloIntVar[][][] Q_ijt_IP;

    public ExpectedDemandModel(ArrayList<Material> rawMaterials, ArrayList<Block> blocks, int T) throws IloException {
        this.rawMaterials = rawMaterials;
        this.blocks = blocks;
        this.T = T;
        // 生成所有切割方案
        allPatterns = new ArrayList<>();
        for (Material rawMaterial : rawMaterials) {
            allPatterns.add(patternsGenerate(rawMaterial, blocks));
        }
        // 构建期望需求矩阵 D_it（直接使用 blocks 中给定的每期需求均值）
        D_it = new int[blocks.size()][T];
        for (int k = 0; k < blocks.size(); k++) {
            ArrayList<Integer> avg = blocks.get(k).getDemandAverage();
            for (int t = 0; t < T; t++) {
                D_it[k][t] = avg.get(t);
            }
        }
        // 分配变量容器
        Y_i = new IloNumVar[rawMaterials.size()];
        Ic_it = new IloNumVar[rawMaterials.size()][T];
        Ie_it = new IloNumVar[blocks.size()][T + 1];
        B_it = new IloNumVar[blocks.size()][T + 1];
        Q_ijt = new IloNumVar[rawMaterials.size()][][];
        Y_i_IP = new IloIntVar[rawMaterials.size()];
        Ic_it_IP = new IloIntVar[rawMaterials.size()][T];
        Ie_it_IP = new IloIntVar[blocks.size()][T + 1];
        B_it_IP = new IloIntVar[blocks.size()][T + 1];
        Q_ijt_IP = new IloIntVar[rawMaterials.size()][][];
    }

    public void buildLP() throws IloException {
        modelLP = new IloCplex();
        // 变量
        for (int i = 0; i < rawMaterials.size(); i++) {
            Y_i[i] = modelLP.numVar(0, Double.MAX_VALUE, "Y" + (i + 1));
            for (int t = 0; t < T; t++) {
                Ic_it[i][t] = modelLP.numVar(0, Double.MAX_VALUE, "Ic" + (i + 1) + "," + (t + 1));
            }
        }
        for (int k = 0; k < blocks.size(); k++) {
            for (int t = 0; t <= T; t++) {
                Ie_it[k][t] = modelLP.numVar(0, Double.MAX_VALUE, "Ie" + (k + 1) + "," + t);
                B_it[k][t] = modelLP.numVar(0, Double.MAX_VALUE, "B" + (k + 1) + "," + t);
            }
        }
        for (int i = 0; i < rawMaterials.size(); i++) {
            Q_ijt[i] = new IloNumVar[allPatterns.get(i).size()][T];
            for (int j = 0; j < allPatterns.get(i).size(); j++) {
                for (int t = 0; t < T; t++) {
                    Q_ijt[i][j][t] = modelLP.numVar(0, Double.MAX_VALUE, "Q" + (i + 1) + "," + (j + 1) + "," + (t + 1));
                }
            }
        }
        // 目标：采购成本 + 原料持有成本 + 切割成本 + 成品持有/缺货成本（全期求和）
        IloNumExpr obj = modelLP.numExpr();
        for (int i = 0; i < rawMaterials.size(); i++) {
            obj = modelLP.sum(obj, modelLP.prod(rawMaterials.get(i).getCost(), Y_i[i]));
            for (int t = 0; t < T; t++) {
                obj = modelLP.sum(obj, modelLP.prod(rawMaterials.get(i).getHold_cost(), Ic_it[i][t]));
                for (int j = 0; j < allPatterns.get(i).size(); j++) {
                    obj = modelLP.sum(obj, modelLP.prod(Parameters.cutCost, Q_ijt[i][j][t]));
                }
            }
        }
        for (int k = 0; k < blocks.size(); k++) {
            for (int t = 0; t <= T; t++) {
                obj = modelLP.sum(obj, modelLP.prod(blocks.get(k).getHold_cost(), Ie_it[k][t]));
                obj = modelLP.sum(obj, modelLP.prod(blocks.get(k).getBack_cost(), B_it[k][t]));
            }
        }
        modelLP.addMinimize(obj);

        // 约束：
        // 1) t=0 的成品期初平衡：B(0) - Ie(0) = 0
        for (int k = 0; k < blocks.size(); k++) {
            IloNumExpr c = modelLP.diff(B_it[k][0], Ie_it[k][0]);
            modelLP.addEq(c, 0, "blkInit" + (k + 1));
        }
        // 2) t=1 的原料平衡：Ic(i,1) = Y(i) + Σ_j Q(i,j,1)
        for (int i = 0; i < rawMaterials.size(); i++) {
            IloNumExpr c = modelLP.diff(Ic_it[i][0], Y_i[i]);
            for (int j = 0; j < allPatterns.get(i).size(); j++) c = modelLP.sum(c, Q_ijt[i][j][0]);
            modelLP.addEq(c, 0, "matBal1_" + (i + 1));
        }
        // 3) t>=2 的原料平衡：Ic(i,t) = Ic(i,t-1) - Σ_j Q(i,j,t)
        for (int i = 0; i < rawMaterials.size(); i++) {
            for (int t = 1; t < T; t++) {
                IloNumExpr c = modelLP.diff(Ic_it[i][t - 1], Ic_it[i][t]);
                for (int j = 0; j < allPatterns.get(i).size(); j++) c = modelLP.diff(c, Q_ijt[i][j][t]);
                modelLP.addEq(c, 0, "matBal_" + (i + 1) + "," + (t + 1));
            }
        }
        // 4) t=1..T 的成品平衡：Σ_{i,j} a_ijk Q(i,j,t) + Ie(k,t-1) - B(k,t-1) - D_it(k,t) = Ie(k,t) - B(k,t)
        int[][][] a_ijk = buildAijk();
        for (int t = 1; t <= T; t++) {
            for (int k = 0; k < blocks.size(); k++) {
                IloNumExpr lhs = modelLP.numExpr();
                // 产出
                for (int i = 0; i < rawMaterials.size(); i++) {
                    for (int j = 0; j < allPatterns.get(i).size(); j++) {
                        lhs = modelLP.sum(lhs, modelLP.prod(a_ijk[i][j][k], Q_ijt[i][j][t - 1]));
                    }
                }
                // 期初库存/缺货
                lhs = modelLP.sum(lhs, Ie_it[k][t - 1]);
                lhs = modelLP.diff(lhs, B_it[k][t - 1]);
                // 减去需求
                lhs = modelLP.diff(lhs, D_it[k][t - 1]);
                IloNumExpr rhs = modelLP.diff(Ie_it[k][t], B_it[k][t]);
                modelLP.addEq(lhs, rhs, "blkBal_" + (k + 1) + ",t" + t);
            }
        }
    }

    public boolean solveLP() throws IloException {
        return modelLP.solve();
    }

    public void exportLP(String path) throws IloException {
        modelLP.exportModel(path);
    }

    // ------------------ IP model ------------------
    public void buildIP() throws IloException {
        modelIP = new IloCplex();
        modelIP.setParam(IloCplex.DoubleParam.TimeLimit, 300);
        modelIP.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        // 变量（整数）
        for (int i = 0; i < rawMaterials.size(); i++) {
            Y_i_IP[i] = modelIP.intVar(0, Integer.MAX_VALUE, "Y" + (i + 1) + ",IP");
            for (int t = 0; t < T; t++) {
                Ic_it_IP[i][t] = modelIP.intVar(0, Integer.MAX_VALUE, "Ic" + (i + 1) + "," + (t + 1) + ",IP");
            }
        }
        for (int k = 0; k < blocks.size(); k++) {
            for (int t = 0; t <= T; t++) {
                Ie_it_IP[k][t] = modelIP.intVar(0, Integer.MAX_VALUE, "Ie" + (k + 1) + "," + t + ",IP");
                B_it_IP[k][t] = modelIP.intVar(0, Integer.MAX_VALUE, "B" + (k + 1) + "," + t + ",IP");
            }
        }
        for (int i = 0; i < rawMaterials.size(); i++) {
            Q_ijt_IP[i] = new IloIntVar[allPatterns.get(i).size()][T];
            for (int j = 0; j < allPatterns.get(i).size(); j++) {
                for (int t = 0; t < T; t++) {
                    Q_ijt_IP[i][j][t] = modelIP.intVar(0, Integer.MAX_VALUE, "Q" + (i + 1) + "," + (j + 1) + "," + (t + 1) + ",IP");
                }
            }
        }
        // 目标
        IloNumExpr obj = modelIP.numExpr();
        for (int i = 0; i < rawMaterials.size(); i++) {
            obj = modelIP.sum(obj, modelIP.prod(rawMaterials.get(i).getCost(), Y_i_IP[i]));
            for (int t = 0; t < T; t++) {
                obj = modelIP.sum(obj, modelIP.prod(rawMaterials.get(i).getHold_cost(), Ic_it_IP[i][t]));
                for (int j = 0; j < allPatterns.get(i).size(); j++) {
                    obj = modelIP.sum(obj, modelIP.prod(Parameters.cutCost, Q_ijt_IP[i][j][t]));
                }
            }
        }
        for (int k = 0; k < blocks.size(); k++) {
            for (int t = 0; t <= T; t++) {
                obj = modelIP.sum(obj, modelIP.prod(blocks.get(k).getHold_cost(), Ie_it_IP[k][t]));
                obj = modelIP.sum(obj, modelIP.prod(blocks.get(k).getBack_cost(), B_it_IP[k][t]));
            }
        }
        modelIP.addMinimize(obj);

        // 约束
        for (int k = 0; k < blocks.size(); k++) {
            IloNumExpr c = modelIP.diff(B_it_IP[k][0], Ie_it_IP[k][0]);
            modelIP.addEq(c, 0, "blkInit" + (k + 1) + ",IP");
        }
        for (int i = 0; i < rawMaterials.size(); i++) {
            IloNumExpr c = modelIP.diff(Ic_it_IP[i][0], Y_i_IP[i]);
            for (int j = 0; j < allPatterns.get(i).size(); j++) c = modelIP.sum(c, Q_ijt_IP[i][j][0]);
            modelIP.addEq(c, 0, "matBal1_" + (i + 1) + ",IP");
        }
        for (int i = 0; i < rawMaterials.size(); i++) {
            for (int t = 1; t < T; t++) {
                IloNumExpr c = modelIP.diff(Ic_it_IP[i][t - 1], Ic_it_IP[i][t]);
                for (int j = 0; j < allPatterns.get(i).size(); j++) c = modelIP.diff(c, Q_ijt_IP[i][j][t]);
                modelIP.addEq(c, 0, "matBal_" + (i + 1) + "," + (t + 1) + ",IP");
            }
        }
        int[][][] a_ijk = buildAijk();
        for (int t = 1; t <= T; t++) {
            for (int k = 0; k < blocks.size(); k++) {
                IloNumExpr lhs = modelIP.numExpr();
                for (int i = 0; i < rawMaterials.size(); i++) {
                    for (int j = 0; j < allPatterns.get(i).size(); j++) {
                        lhs = modelIP.sum(lhs, modelIP.prod(a_ijk[i][j][k], Q_ijt_IP[i][j][t - 1]));
                    }
                }
                lhs = modelIP.sum(lhs, Ie_it_IP[k][t - 1]);
                lhs = modelIP.diff(lhs, B_it_IP[k][t - 1]);
                lhs = modelIP.diff(lhs, D_it[k][t - 1]);
                IloNumExpr rhs = modelIP.diff(Ie_it_IP[k][t], B_it_IP[k][t]);
                modelIP.addEq(lhs, rhs, "blkBal_" + (k + 1) + ",t" + t + ",IP");
            }
        }
    }

    public boolean solveIP() throws IloException {
        return modelIP.solve();
    }

    public void exportIP(String path) throws IloException {
        modelIP.exportModel(path);
    }
    
    public DeterministicSolution extractSolution() throws IloException {
        DeterministicSolution solution = new DeterministicSolution(rawMaterials, blocks, allPatterns, T);
        
        // 提取目标函数值
        solution.obj_value_IP = modelIP.getObjValue();
        
        // 提取采购决策
        solution.Y_value_IP = modelIP.getValues(Y_i_IP);
        
        // 提取库存变量
        for(int i = 0; i < rawMaterials.size(); i++) {
            for(int t = 0; t < T; t++) {
                solution.Ic_value_IP[i][t] = modelIP.getValue(Ic_it_IP[i][t]);
            }
        }
        
        for(int k = 0; k < blocks.size(); k++) {
            for(int t = 0; t <= T; t++) {
                solution.Ie_value_IP[k][t] = modelIP.getValue(Ie_it_IP[k][t]);
                solution.B_value_IP[k][t] = modelIP.getValue(B_it_IP[k][t]);
            }
        }
        
        // 提取切割决策
        for(int i = 0; i < rawMaterials.size(); i++) {
            for(int j = 0; j < allPatterns.get(i).size(); j++) {
                for(int t = 0; t < T; t++) {
                    solution.Q_value_IP[i][j][t] = modelIP.getValue(Q_ijt_IP[i][j][t]);
                }
            }
        }
        
        return solution;
    }

    public int[][][] buildAijk() {
        int[][][] a = new int[rawMaterials.size()][][];
        for (int i = 0; i < rawMaterials.size(); i++) {
            a[i] = new int[allPatterns.get(i).size()][];
            for (int j = 0; j < allPatterns.get(i).size(); j++) {
                a[i][j] = new int[blocks.size()];
                HashMap<Integer, Integer> scheme = allPatterns.get(i).get(j).scheme;
                for (int k = 0; k < blocks.size(); k++) {
                    a[i][j][k] = scheme.getOrDefault(blocks.get(k).getLength(), 0);
                }
            }
        }
        return a;
    }

    public static ArrayList<Pattern> patternsGenerate(Material curMaterial, ArrayList<Block> curBlocks) {
        ArrayList<Integer> blockLengths = new ArrayList<>();
        for (Block b : curBlocks) blockLengths.add(b.getLength());
        ArrayList<ArrayList<Integer>> allPatterns = new ArrayList<>();
        generate(curMaterial.getLength(), blockLengths, 0, new ArrayList<>(), allPatterns);
        ArrayList<Pattern> patterns = new ArrayList<>();
        for (ArrayList<Integer> pat : allPatterns) {
            Pattern pattern = new Pattern();
            HashMap<Integer, Integer> pattern_map = new HashMap<>();
            for (Integer len : pat) pattern_map.put(len, pattern_map.getOrDefault(len, 0) + 1);
            pattern.type = curMaterial.getType();
            pattern.scheme = pattern_map;
            patterns.add(pattern);
        }
        return patterns;
    }

    private static void generate(int remainingLength, ArrayList<Integer> segmentLengths, int start,
                                 ArrayList<Integer> currentScheme, ArrayList<ArrayList<Integer>> result) {
        if (remainingLength == 0) {
            result.add(new ArrayList<>(currentScheme));
            return;
        }
        for (int i = start; i < segmentLengths.size(); i++) {
            if (segmentLengths.get(i) <= remainingLength) {
                currentScheme.add(segmentLengths.get(i));
                generate(remainingLength - segmentLengths.get(i), segmentLengths, i, currentScheme, result);
                currentScheme.remove(currentScheme.size() - 1);
            }
        }
    }
}