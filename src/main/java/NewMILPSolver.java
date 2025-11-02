import ilog.concert.*;
import ilog.cplex.IloCplex;
import java.io.BufferedReader;
import java.io.FileReader;
import java.util.ArrayList;
import java.util.HashMap;

/**
 * 新的MILPSolver - 基于ExpectedDemandModel的结构
 * 决策变量和建模方式与ExpectedDemandModel保持一致
 * 支持多场景随机规划
 */
public class NewMILPSolver {
    
    // 基本数据结构
    public ArrayList<Material> rawMaterials;  // 原材料集合
    public ArrayList<Block> blocks;  // 零件集合
    public ArrayList<ArrayList<Pattern>> allPatterns;  // 所有切割方案
    public int T; // 总周期数
    
    // 场景相关
    public ArrayList<Scenario> scenarios;  // 场景集合
    
    // CPLEX模型
    public IloCplex model;
    
    // 决策变量（与ExpectedDemandModel结构一致，但增加场景维度）
    public IloIntVar[] Y_i;  // 第一期采购量 [i] (与ExpectedDemandModel一致)
    public IloIntVar[][][] Ic_ist;  // 原材料库存 [i][s][t] (增加场景维度)
    public IloIntVar[][][] Ie_kst;  // 成品库存 [k][s][t] (增加场景维度)
    public IloIntVar[][][] B_kst;   // 成品缺货 [k][s][t] (增加场景维度)
    public IloIntVar[][][][] Q_ijst; // 切割方案 [i][j][s][t] (增加场景维度)
    
    // 场景数据结构
    public static class Scenario {
        public String id;
        public double prob;
        public int[][] D_kt;  // 场景s中成品k在t期的需求 [k][t]
        
        public Scenario(String id, double prob, int numBlocks, int T) {
            this.id = id;
            this.prob = prob;
            this.D_kt = new int[numBlocks][T];
        }
    }
    
    public NewMILPSolver() {
        this.scenarios = new ArrayList<>();
    }
    
    // 读取数据文件（复用原有逻辑）
    public void loadMaterials(String filepath) throws Exception {
        BufferedReader br = new BufferedReader(new FileReader(filepath));
        String line;
        int lineNo = 0;
        rawMaterials = new ArrayList<>();
        
        while((line = br.readLine()) != null){
            lineNo++;
            line = line.trim();
            if(line.isEmpty() || line.startsWith("#")) continue;
            String[] a = line.split(",");
            try {
                if(a.length < 4){
                    System.out.println("跳过原材料行(列不足) line " + lineNo + ": " + line);
                    continue;
                }
                String type = a[0].trim();
                int len = Integer.parseInt(a[1].trim());
                double cost = Double.parseDouble(a[2].trim());
                double hold = Double.parseDouble(a[3].trim());
                rawMaterials.add(new Material(type, len, cost, hold));
            } catch(Exception ex) {
                System.out.println("跳过原材料表头或无法解析行 " + lineNo + ": " + line);
                continue;
            }
        }
        br.close();
        System.out.println("读取原材料: 共 " + rawMaterials.size() + " 种");
    }
    
    public void loadBlocks(String filepath) throws Exception {
        BufferedReader br = new BufferedReader(new FileReader(filepath));
        String line;
        int lineNo = 0;
        blocks = new ArrayList<>();
        T = 5; // 默认周期数
        
        while((line = br.readLine()) != null){
            lineNo++;
            line = line.trim();
            if(line.isEmpty() || line.startsWith("#")) continue;
            String[] a = line.split(",");
            try {
                if(a.length < 5 + T) {
                    System.out.println("跳过成品行(列不足) line " + lineNo + ": " + line);
                    continue;
                }
                int index = Integer.parseInt(a[0].trim());
                String type = a[1].trim();
                int len = Integer.parseInt(a[2].trim());
                double hold = Double.parseDouble(a[3].trim());
                double back = Double.parseDouble(a[4].trim());
                ArrayList<Integer> meanList = new ArrayList<>();
                for(int t=0; t<T; t++){
                    int val = Integer.parseInt(a[5 + t].trim());
                    meanList.add(val);
                }
                blocks.add(new Block(index, type, len, hold, back, meanList));
            } catch(Exception ex){
                System.out.println("跳过成品表头或无法解析行 " + lineNo + ": " + line);
                continue;
            }
        }
        br.close();
        System.out.println("读取成品: 共 " + blocks.size() + " 种");
    }
    
    // 生成切割方案（与ExpectedDemandModel一致）
    public void generatePatterns() {
        allPatterns = new ArrayList<>();
        for (Material rawMaterial : rawMaterials) {
            allPatterns.add(patternsGenerate(rawMaterial, blocks));
        }
        System.out.println("生成切割方案完成");
    }
    
    // 生成场景（使用ScenarioTree）
    public void generateScenarios(int numScenarios, ScenarioTree tree) throws Exception {
        scenarios.clear();
        
        // 构建期望需求矩阵
        ArrayList<HashMap<Integer, Integer>> meanDemands = new ArrayList<>();
        for(int t = 0; t < T; t++) {
            HashMap<Integer, Integer> demandMap = new HashMap<>();
            for(Block block : blocks) {
                demandMap.put(block.getLength(), block.getDemandAverage().get(t));
            }
            meanDemands.add(demandMap);
        }
        
        // 生成场景
        for(int s = 0; s < numScenarios; s++) {
            ArrayList<HashMap<Integer, Integer>> scenarioPath = tree.generateNormal(meanDemands);
            Scenario scenario = new Scenario("S" + (s+1), 1.0 / numScenarios, blocks.size(), T);
            
            // 填充场景需求数据
            for(int t = 0; t < T; t++) {
                HashMap<Integer, Integer> demandMap = scenarioPath.get(t);
                for(int k = 0; k < blocks.size(); k++) {
                    scenario.D_kt[k][t] = demandMap.getOrDefault(blocks.get(k).getLength(), 0);
                }
            }
            scenarios.add(scenario);
        }
        
        System.out.println("生成场景数: " + scenarios.size());
    }
    
    // 构建并求解MILP模型
    public NewMILPSolution buildAndSolve() throws Exception {
        model = new IloCplex();
        model.setParam(IloCplex.DoubleParam.TimeLimit, 600);
        model.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        
        int numScenarios = scenarios.size();
        int numRawMaterials = rawMaterials.size();
        int numBlocks = blocks.size();
        
        // 分配变量（与ExpectedDemandModel结构一致）
        Y_i = new IloIntVar[numRawMaterials];
        Ic_ist = new IloIntVar[numRawMaterials][numScenarios][T];
        Ie_kst = new IloIntVar[numBlocks][numScenarios][T + 1];
        B_kst = new IloIntVar[numBlocks][numScenarios][T + 1];
        Q_ijst = new IloIntVar[numRawMaterials][][][];
        
        // 初始化切割方案变量维度
        for(int i = 0; i < numRawMaterials; i++) {
            Q_ijst[i] = new IloIntVar[allPatterns.get(i).size()][numScenarios][T];
        }
        
        // 创建变量
        createVariables();
        
        // 构建目标函数
        buildObjective();
        
        // 构建约束
        buildConstraints();
        
        System.out.println("模型构建完成，开始求解...");
        System.out.println("场景数: " + scenarios.size() + ", 原材料数: " + rawMaterials.size() + ", 成品数: " + blocks.size() + ", 周期数: " + T);
        
        boolean solved = model.solve();
        
        NewMILPSolution solution = null;
        if(solved) {
            double objValue = model.getObjValue();
            System.out.println("求解成功！目标值 = " + objValue);
            
            if(objValue == 0.0) {
                System.out.println("警告：目标值为0，可能存在问题");
            }
            
            printSolution();
            
            // 提取解决方案
            solution = extractSolution();
            System.out.println("解决方案提取完成，目标值 = " + solution.obj_value);
            
        } else {
            System.out.println("模型求解失败，状态 = " + model.getStatus());
            System.out.println("可能的原因：无可行解、时间限制、内存不足等");
        }
        
        model.end();
        return solution;
    }
    
    // 创建变量（与ExpectedDemandModel结构一致）
    private void createVariables() throws IloException {
        int numScenarios = scenarios.size();
        int numRawMaterials = rawMaterials.size();
        int numBlocks = blocks.size();
        
        // 第一期采购变量 Y_i（所有场景共享，与ExpectedDemandModel一致）
        for(int i = 0; i < numRawMaterials; i++) {
            Y_i[i] = model.intVar(0, Integer.MAX_VALUE, "Y_" + (i + 1));
        }
        
        // 原材料库存变量 Ic_ist
        for(int i = 0; i < numRawMaterials; i++) {
            for(int s = 0; s < numScenarios; s++) {
                for(int t = 0; t < T; t++) {
                    Ic_ist[i][s][t] = model.intVar(0, Integer.MAX_VALUE, 
                        "Ic_" + (i + 1) + "_" + scenarios.get(s).id + "_" + (t + 1));
                }
            }
        }
        
        // 成品库存和缺货变量 Ie_kst, B_kst
        for(int k = 0; k < numBlocks; k++) {
            for(int s = 0; s < numScenarios; s++) {
                for(int t = 0; t <= T; t++) {
                    Ie_kst[k][s][t] = model.intVar(0, Integer.MAX_VALUE, 
                        "Ie_" + (k + 1) + "_" + scenarios.get(s).id + "_" + t);
                    B_kst[k][s][t] = model.intVar(0, Integer.MAX_VALUE, 
                        "B_" + (k + 1) + "_" + scenarios.get(s).id + "_" + t);
                }
            }
        }
        
        // 切割方案变量 Q_ijst
        for(int i = 0; i < numRawMaterials; i++) {
            for(int j = 0; j < allPatterns.get(i).size(); j++) {
                for(int s = 0; s < numScenarios; s++) {
                    for(int t = 0; t < T; t++) {
                        Q_ijst[i][j][s][t] = model.intVar(0, Integer.MAX_VALUE, 
                            "Q_" + (i + 1) + "_" + (j + 1) + "_" + scenarios.get(s).id + "_" + (t + 1));
                    }
                }
            }
        }
    }
    
    // 构建目标函数（与ExpectedDemandModel结构一致）
    private void buildObjective() throws IloException {
        IloLinearNumExpr obj = model.linearNumExpr();
        
        // 第一期采购成本（所有场景共享）
        for(int i = 0; i < rawMaterials.size(); i++) {
            obj.addTerm(rawMaterials.get(i).getCost(), Y_i[i]);
        }
        
        // 期望运营成本（所有场景的期望）
        for(Scenario s : scenarios) {
            double prob = s.prob;
            int sIndex = scenarios.indexOf(s);
            
            // 原材料库存成本
            for(int i = 0; i < rawMaterials.size(); i++) {
                for(int t = 0; t < T; t++) {
                    obj.addTerm(prob * rawMaterials.get(i).getHold_cost(), Ic_ist[i][sIndex][t]);
                }
            }
            
            // 切割成本
            for(int i = 0; i < rawMaterials.size(); i++) {
                for(int j = 0; j < allPatterns.get(i).size(); j++) {
                    for(int t = 0; t < T; t++) {
                        obj.addTerm(prob * Parameters.cutCost, Q_ijst[i][j][sIndex][t]);
                    }
                }
            }
            
            // 成品库存和缺货成本
            for(int k = 0; k < blocks.size(); k++) {
                for(int t = 0; t <= T; t++) {
                    obj.addTerm(prob * blocks.get(k).getHold_cost(), Ie_kst[k][sIndex][t]);
                    obj.addTerm(prob * blocks.get(k).getBack_cost(), B_kst[k][sIndex][t]);
                }
            }
        }
        
        model.addMinimize(obj);
    }
    
    // 构建约束（与ExpectedDemandModel结构一致）
    private void buildConstraints() throws IloException {
        int numScenarios = scenarios.size();
        int numRawMaterials = rawMaterials.size();
        int numBlocks = blocks.size();
        int[][][] a_ijk = buildAijk();
        
        // 为每个场景构建约束
        for(int s = 0; s < numScenarios; s++) {
            Scenario scenario = scenarios.get(s);
            
            // 1) t=0 的成品期初平衡：B_kst(0) - Ie_kst(0) = 0
            for(int k = 0; k < numBlocks; k++) {
                IloLinearNumExpr expr = model.linearNumExpr();
                expr.addTerm(1.0, B_kst[k][s][0]);
                expr.addTerm(-1.0, Ie_kst[k][s][0]);
                model.addEq(expr, 0.0, "blkInit_" + (k + 1) + "_" + scenario.id);
            }
            
            // 2) t=1 的原料平衡：Ic_ist(0) = Y_i - Σ_j Q_ijst(0)
            for(int i = 0; i < numRawMaterials; i++) {
                IloLinearNumExpr expr = model.linearNumExpr();
                expr.addTerm(1.0, Y_i[i]);
                expr.addTerm(-1.0, Ic_ist[i][s][0]);
                for(int j = 0; j < allPatterns.get(i).size(); j++) {
                    expr.addTerm(-1.0, Q_ijst[i][j][s][0]);
                }
                model.addEq(expr, 0.0, "matBal1_" + (i + 1) + "_" + scenario.id);
            }
            
            // 3) t>=2 的原料平衡：Ic_ist(t) = Ic_ist(t-1) - Σ_j Q_ijst(t)
            for(int i = 0; i < numRawMaterials; i++) {
                for(int t = 1; t < T; t++) {
                    IloLinearNumExpr expr = model.linearNumExpr();
                    expr.addTerm(1.0, Ic_ist[i][s][t - 1]);
                    expr.addTerm(-1.0, Ic_ist[i][s][t]);
                    for(int j = 0; j < allPatterns.get(i).size(); j++) {
                        expr.addTerm(-1.0, Q_ijst[i][j][s][t]);
                    }
                    model.addEq(expr, 0.0, "matBal_" + (i + 1) + "_" + (t + 1) + "_" + scenario.id);
                }
            }
            
            // 4) t=1..T 的成品平衡：Σ_{i,j} a_ijk Q_ijst(t-1) + Ie_kst(t-1) - B_kst(t-1) - D_kt(t-1) = Ie_kst(t) - B_kst(t)
            for(int t = 1; t <= T; t++) {
                for(int k = 0; k < numBlocks; k++) {
                    IloLinearNumExpr lhs = model.linearNumExpr();
                    
                    // 产出项
                    for(int i = 0; i < numRawMaterials; i++) {
                        for(int j = 0; j < allPatterns.get(i).size(); j++) {
                            lhs.addTerm(a_ijk[i][j][k], Q_ijst[i][j][s][t - 1]);
                        }
                    }
                    
                    // 期初库存/缺货
                    lhs.addTerm(1.0, Ie_kst[k][s][t - 1]);
                    lhs.addTerm(-1.0, B_kst[k][s][t - 1]);
                    
                    // 期末库存/缺货
                    lhs.addTerm(-1.0, Ie_kst[k][s][t]);
                    lhs.addTerm(1.0, B_kst[k][s][t]);
                    
                    // 需求项（RHS）
                    model.addEq(lhs, scenario.D_kt[k][t - 1], 
                        "blkBal_" + (k + 1) + "_t" + t + "_" + scenario.id);
                }
            }
        }
    }
    
    // 构建a_ijk矩阵（与ExpectedDemandModel一致）
    private int[][][] buildAijk() {
        int numRawMaterials = rawMaterials.size();
        int numBlocks = blocks.size();
        int[][][] a = new int[numRawMaterials][][];
        
        for(int i = 0; i < numRawMaterials; i++) {
            a[i] = new int[allPatterns.get(i).size()][];
            for(int j = 0; j < allPatterns.get(i).size(); j++) {
                a[i][j] = new int[numBlocks];
                HashMap<Integer, Integer> scheme = allPatterns.get(i).get(j).scheme;
                for(int k = 0; k < numBlocks; k++) {
                    a[i][j][k] = scheme.getOrDefault(blocks.get(k).getLength(), 0);
                }
            }
        }
        return a;
    }
    
    // 生成切割方案（与ExpectedDemandModel一致）
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
    
    // 提取解决方案
    public NewMILPSolution extractSolution() throws IloException {
        NewMILPSolution solution = new NewMILPSolution(rawMaterials, blocks, allPatterns, scenarios.size(), T);
        
        solution.obj_value = model.getObjValue();
        
        // 提取采购变量
        for(int i = 0; i < rawMaterials.size(); i++) {
            solution.Y_value[i] = model.getValue(Y_i[i]);
        }
        
        // 提取原材料库存变量
        for(int i = 0; i < rawMaterials.size(); i++) {
            for(int s = 0; s < scenarios.size(); s++) {
                for(int t = 0; t < T; t++) {
                    solution.Ic_value[i][s][t] = model.getValue(Ic_ist[i][s][t]);
                }
            }
        }
        
        // 提取成品库存和缺货变量
        for(int k = 0; k < blocks.size(); k++) {
            for(int s = 0; s < scenarios.size(); s++) {
                for(int t = 0; t <= T; t++) {
                    solution.Ie_value[k][s][t] = model.getValue(Ie_kst[k][s][t]);
                    solution.B_value[k][s][t] = model.getValue(B_kst[k][s][t]);
                }
            }
        }
        
        // 提取切割方案变量
        for(int i = 0; i < rawMaterials.size(); i++) {
            for(int j = 0; j < allPatterns.get(i).size(); j++) {
                for(int s = 0; s < scenarios.size(); s++) {
                    for(int t = 0; t < T; t++) {
                        solution.Q_value[i][j][s][t] = model.getValue(Q_ijst[i][j][s][t]);
                    }
                }
            }
        }
        
        return solution;
    }
    
    // 模拟函数 - 完全仿照Functions.simulator的形式
    public double simulator(ScenarioTree scenarioTree, NewMILPSolution solution) throws IloException {
        int numPath = Parameters.simulatePaths;
        
        // 期初采购成本（所有路径共享，只计算一次）
        double procurementCost = 0;
        for(int i = 0; i < rawMaterials.size(); i++) {
            procurementCost += solution.Y_value[i] * rawMaterials.get(i).getCost();
        }
        
        double operationalCost = 0;
        
        // 为每条路径进行模拟
        for(int pathIndex = 0; pathIndex < numPath; pathIndex++) {
            // 生成新的场景路径
            ArrayList<Integer> path = scenarioTree.samplePath();
            
            // 初始化库存状态
            double[] materialInventory = new double[rawMaterials.size()];
            double[] blockInventory = new double[blocks.size()];
            
            // 设置第一期初的原材料库存（基于solution中的采购决策）
            for(int i = 0; i < rawMaterials.size(); i++) {
                materialInventory[i] = solution.Y_value[i];
            }
            
            // 切割生产阶段 (Period 0)
            for(int i = 0; i < rawMaterials.size(); i++) {
                for(int j = 0; j < allPatterns.get(i).size(); j++) {
                    // 获取切割方案使用量 - 使用第一个场景的决策作为代表
                    double cutAmount = solution.Q_value[i][j][0][0];
                    
                    // 更新原材料库存
                    materialInventory[i] -= cutAmount;
                    
                    // 添加切割成本
                    operationalCost += cutAmount * Parameters.cutCost;
                    
                    // 根据切割方案生产零件
                    int[][][] a_ijk = buildAijk();
                    for(int k = 0; k < blocks.size(); k++) {
                        double producedAmount = cutAmount * a_ijk[i][j][k];
                        blockInventory[k] += producedAmount;
                    }
                }
            }
            
            // 模拟每个周期的运营 (Period 1 to T-1)
            for(int t = 1; t < T; t++) {
                // 获取当前周期的需求
                HashMap<Integer, Integer> currentDemand = scenarioTree.tree.get(t-1).getCurLayer().get(path.get(t-1));
                
                // 切割生产阶段
                for(int i = 0; i < rawMaterials.size(); i++) {
                    for(int j = 0; j < allPatterns.get(i).size(); j++) {
                        // 获取切割方案使用量 - 使用第一个场景的决策作为代表
                        double cutAmount = solution.Q_value[i][j][0][t];
                        
                        // 更新原材料库存
                        materialInventory[i] -= cutAmount;
                        
                        // 添加切割成本
                        operationalCost += cutAmount * Parameters.cutCost;
                        
                        // 根据切割方案生产零件
                        int[][][] a_ijk = buildAijk();
                        for(int k = 0; k < blocks.size(); k++) {
                            double producedAmount = cutAmount * a_ijk[i][j][k];
                            blockInventory[k] += producedAmount;
                        }
                    }
                }
                
                // 需求满足阶段
                for(int k = 0; k < blocks.size(); k++) {
                    int demand = currentDemand.getOrDefault(blocks.get(k).getLength(), 0);
                    
                    // 优先用库存满足需求
                    double satisfiedFromInventory = Math.min(demand, blockInventory[k]);
                    blockInventory[k] -= satisfiedFromInventory;
                    
                    // 剩余需求直接购买成品（缺货成本）
                    double unmetDemand = demand - satisfiedFromInventory;
                    if(unmetDemand > 0) {
                        operationalCost += unmetDemand * blocks.get(k).getBack_cost();
                    }
                }
                
                // 库存持有成本
                for(int i = 0; i < rawMaterials.size(); i++) {
                    operationalCost += materialInventory[i] * rawMaterials.get(i).getHold_cost();
                }
                
                for(int k = 0; k < blocks.size(); k++) {
                    operationalCost += blockInventory[k] * blocks.get(k).getHold_cost();
                }
            }
            
            // 期末需求处理 (Last Period T-1)
            HashMap<Integer, Integer> currentDemand = scenarioTree.tree.get(T-1).getCurLayer().get(path.get(T-1));
            for(int k = 0; k < blocks.size(); k++) {
                int demand = currentDemand.getOrDefault(blocks.get(k).getLength(), 0);
                
                // 优先用库存满足需求
                double satisfiedFromInventory = Math.min(demand, blockInventory[k]);
                blockInventory[k] -= satisfiedFromInventory;
                
                // 剩余需求直接购买成品（缺货成本）
                double unmetDemand = demand - satisfiedFromInventory;
                if(unmetDemand > 0) {
                    operationalCost += unmetDemand * blocks.get(k).getBack_cost();
                }
            }
        }
        
        // 返回期初采购成本加上平均运营成本
        return procurementCost + (operationalCost / numPath);
    }
    
    // 打印解决方案
    private void printSolution() throws IloException {
        System.out.println("=== 解决方案 ===");
        
        // 打印采购量
        System.out.println("原材料采购量:");
        for(int i = 0; i < rawMaterials.size(); i++) {
            double val = model.getValue(Y_i[i]);
            if(val > 1e-6) {
                System.out.printf("  原材料 %s: %.0f\n", rawMaterials.get(i).getType(), val);
            }
        }
        
        // 打印第一个场景的部分结果
        if(scenarios.size() > 0) {
            Scenario s0 = scenarios.get(0);
            System.out.println("=== 场景 " + s0.id + " 的部分结果示例 ===");
            
            // 原材料库存
            for(int i = 0; i < rawMaterials.size(); i++) {
                for(int t = 0; t < T; t++) {
                    double val = model.getValue(Ic_ist[i][0][t]);
                    if(val > 1e-6) {
                        System.out.printf("  原材料 %s 第%d期库存: %.0f\n", 
                            rawMaterials.get(i).getType(), t + 1, val);
                    }
                }
            }
            
            // 成品库存/缺货
            for(int k = 0; k < blocks.size(); k++) {
                for(int t = 1; t <= T; t++) {
                    double invVal = model.getValue(Ie_kst[k][0][t]);
                    double backVal = model.getValue(B_kst[k][0][t]);
                    if(invVal > 1e-6 || backVal > 1e-6) {
                        System.out.printf("  成品 %d 第%d期: 库存=%.0f, 缺货=%.0f\n", 
                            blocks.get(k).getLength(), t, invVal, backVal);
                    }
                }
            }
        }
    }
    
    // 主函数用于测试
    public static void main(String[] args) {
        String materialFile = "src/data/materials_demo.csv";
        String blockFile = "src/data/blocks_index.csv";
        int numScenarios = 1000;  // 使用较少的场景数进行测试
        
        NewMILPSolver solver = new NewMILPSolver();
        try {
            solver.loadMaterials(materialFile);
            solver.loadBlocks(blockFile);
            solver.generatePatterns();
            
            // 生成场景
            ScenarioTree tree = new ScenarioTree(solver.blocks);
            
            // 构建场景树
            ArrayList<HashMap<Integer, Integer>> demandAverage = new ArrayList<>();
            for(int i=0;i<solver.T;i++){
                HashMap<Integer, Integer> curDemandAverage = new HashMap<>();
                for (Block block : solver.blocks) {
                    curDemandAverage.put(block.getLength(), block.getDemandAverage().get(i));
                }
                demandAverage.add(curDemandAverage);
            }
            tree.buildTree(Parameters.nodeNumPerLayer, solver.T, demandAverage);
            
            solver.generateScenarios(numScenarios, tree);
            
            // 求解
            NewMILPSolution solution = solver.buildAndSolve();
            
            // 进行模拟评估
            if(solution != null) {
                double simulatedCost = solver.simulator(tree, solution);
                System.out.println("模拟评估成本 = " + simulatedCost);
            } else {
                System.out.println("模型求解失败，无法进行模拟评估");
            }
            
        } catch(Exception ex) {
            System.err.println("发生异常: " + ex.getMessage());
            ex.printStackTrace();
        }
    }
}
