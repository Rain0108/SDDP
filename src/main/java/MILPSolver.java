/**
 * MultiPeriodCuttingMILP_Final.java
 *
 * 修正版（中文注释）：
 *  - 修复 CSV 读取（支持带/不带表头）
 *  - patternsGenerate 输出 scheme 的 key 为成品索引（block.index）
 *  - 生成场景后打印每条场景的每期需求（方便确认不是全 0）
 *  - 生成 pattern 后打印每个 pattern 的 scheme（便于检查）
 *  - 保留原材料库存平衡与成品平衡约束
 *
 * 使用：
 * javac -cp .:/path/to/concert.jar:/path/to/cplex.jar MultiPeriodCuttingMILP_Final.java
 * java  -cp .:/path/to/concert.jar:/path/to/cplex.jar MultiPeriodCuttingMILP_Final
 *
 * 注意：请确保 ScenarioTree.generateNormal(...) 可用且与本代码的 meanList 格式兼容。
 */

import ilog.concert.*;
import ilog.cplex.*;
import java.io.*;
import java.util.*;


/** 场景结构（本文件内部使用） */
class Scenario {
    String id;
    double prob;
    ArrayList<HashMap<Integer,Integer>> demands; // demands.get(t-1).get(blockIndex)
    public Scenario(String id, double prob, ArrayList<HashMap<Integer,Integer>> demands){
        this.id = id; this.prob = prob; this.demands = demands;
    }
}

public class MILPSolver {

    static final int T = 5; // 默认周期数（与 Block.csv 中列数对应）
    ArrayList<Material> materials = new ArrayList<>();
    ArrayList<Block> blocks = new ArrayList<>();
    ArrayList<Scenario> scenarios = new ArrayList<>();
    HashMap<String, ArrayList<Pattern>> patternsByRaw = new HashMap<>();

    IloCplex cplex;

    // 变量映射
    Map<String, IloNumVar> Y;  // 原材料第一期采购量 Y[type]
    Map<String, Map<Integer, IloNumVar[]>> I;  // I[scenId][blockIndex][t]
    Map<String, Map<Integer, IloNumVar[]>> B;  // B[scenId][blockIndex][t]
    Map<String, Map<String, List<IloNumVar[]>>> Q; // Q[scenId][rawType] -> list of pattern arrays (每个pattern一个数组[1..T])
    Map<String, Map<String, IloNumVar[]>> R_I; // R_I[scenId][rawType][t] 原材料库存

    public MILPSolver() {}

    /* -------------------- CSV 读取（更鲁棒） -------------------- */

    /**
     * 读取原材料文件 RawMaterial.csv
     * 格式：type,length,cost,hold_cost
     * 支持有或没有表头；自动跳过无法解析的行
     */
    public void loadMaterials(String filepath) throws Exception {
        BufferedReader br = new BufferedReader(new FileReader(filepath));
        String line;
        int lineNo = 0;
        while((line = br.readLine()) != null){
            lineNo++;
            line = line.trim();
            if(line.isEmpty() || line.startsWith("#")) continue;
            String[] a = line.split(",");
            // 尝试解析第一个字段为 type；如果是文字表头则跳过
            try {
                // 第一个字段当字符串保存（即使为数字也可以）
                String type = a[0].trim();
                // 需要至少 4 列
                if(a.length < 4){
                    System.out.println("跳过原材料行(列不足) line " + lineNo + ": " + line);
                    continue;
                }
                int len = Integer.parseInt(a[1].trim());
                double cost = Double.parseDouble(a[2].trim());
                double hold = Double.parseDouble(a[3].trim());
                materials.add(new Material(type, len, cost, hold));
            } catch(Exception ex) {
                // 很可能是表头，跳过
                System.out.println("跳过原材料表头或无法解析行 " + lineNo + ": " + line);
                continue;
            }
        }
        br.close();
        System.out.println("读取原材料: 共 " + materials.size() + " 种");
    }

    /**
     * 读取成品文件 Block.csv
     * 格式：index,type,length,hold_cost,back_cost,mean_t1,...,mean_tT
     * 支持有/无表头
     */
    public void loadBlocks(String filepath) throws Exception {
        BufferedReader br = new BufferedReader(new FileReader(filepath));
        String line;
        int lineNo = 0;
        while((line = br.readLine()) != null){
            lineNo++;
            line = line.trim();
            if(line.isEmpty() || line.startsWith("#")) continue;
            String[] a = line.split(",");
            try {
                if(a.length < 5 + T - 1) {
                    // a.length should be >= 5 + (T-1) because indices start at 0? to be safe:
                    if(a.length < 5 + T) {
                        System.out.println("跳过成品行(列不足) line " + lineNo + ": " + line);
                        continue;
                    }
                }
                // 解析
                int index = Integer.parseInt(a[0].trim());
                String type = a[1].trim();
                int len = Integer.parseInt(a[2].trim());
                double hold = Double.parseDouble(a[3].trim());
                double back = Double.parseDouble(a[4].trim());
                ArrayList<Integer> meanList = new ArrayList<>();
                // 期均值在列 5..(5+T-1)
                for(int t=0; t<T; t++){
                    int val = Integer.parseInt(a[5 + t].trim());
                    meanList.add(val);
                }
                blocks.add(new Block(index, type, len, hold, back, meanList));
            } catch(Exception ex){
                // 表头或无法解析行
                System.out.println("跳过成品表头或无法解析行 " + lineNo + ": " + line);
                continue;
            }
        }
        br.close();
        System.out.println("读取成品: 共 " + blocks.size() + " 种");
    }

    /* -------------------- 切割方案生成（scheme 的 key 为 block.index） -------------------- */

    /**
     * patternsGenerate：基于原来提供的生成逻辑，但把 scheme 的 key 改为 block.index
     * 对于每个原材料，穷举所有由若干成品长度组成、刚好等于原材料长度的组合（允许重复）
     */
    public static ArrayList<Pattern> patternsGenerate(Material curMaterial, ArrayList<Block> curBlocks){
        ArrayList<Integer> blockLengths = new ArrayList<>();
        for(Block b : curBlocks) blockLengths.add(b.length);

        ArrayList<ArrayList<Integer>> allPatterns = new ArrayList<>();
        generate(curMaterial.length, blockLengths, 0, new ArrayList<>(), allPatterns);

        // 先建立长度 -> 第一个匹配的 block.index 映射（保证确定性）
        HashMap<Integer, Integer> lenToIndex = new HashMap<>();
        for(Block b : curBlocks){
            if(!lenToIndex.containsKey(b.length)){
                lenToIndex.put(b.length, b.index);
            }
        }

        ArrayList<Pattern> patterns = new ArrayList<>();
        for (ArrayList<Integer> allPattern : allPatterns) {
            Pattern pattern = new Pattern();
            HashMap<Integer, Integer> pattern_map = new HashMap<>();
            for (Integer lengthVal : allPattern) {
                // 通过长度找到对应的 block.index（若无则跳过该长度）
                if(lenToIndex.containsKey(lengthVal)){
                    int idx = lenToIndex.get(lengthVal);
                    pattern_map.put(idx, pattern_map.getOrDefault(idx, 0) + 1);
                } else {
                    // 没有对应的成品长度，理论上不应该发生，打印警告
                    System.out.println("警告：切割方案中发现无法匹配的成品长度 " + lengthVal + " (原料 " + curMaterial.type + ")");
                }
            }
            pattern.type = curMaterial.type;
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

    /* -------------------- 场景生成：调用用户的 ScenarioTree.generateNormal -------------------- */

    /**
     * 生成 numScenarios 条场景，每条场景调用一次 tree.generateNormal(meanList)
     * meanList: ArrayList<HashMap<blockIndex, meanDemand>> 长度为 T（期数）
     */
    public void generateScenarios(int numScenarios, ScenarioTree tree) throws Exception {
        // 1) 构造 meanList（长度 T），每项为 blockIndex -> mean
        ArrayList<HashMap<Integer,Integer>> meanList = new ArrayList<>();
        for(int t=0; t<T; t++){
            HashMap<Integer,Integer> map = new HashMap<>();
            for(Block b : blocks){
                // blocks 中 demandAverage 存为 ArrayList<Integer>，索引 t 对应期 t+1
                map.put(b.index, b.demandAverage.get(t));
            }
            meanList.add(map);
        }

        // 2) 生成场景路径（每次调用生成一条路径）
        for(int s=0; s<numScenarios; s++){
            ArrayList<HashMap<Integer,Integer>> path = tree.generateNormal(meanList);
            // 简单验证返回的 path 长度是否等于 T
            if(path == null || path.size() != T){
                throw new RuntimeException("ScenarioTree.generateNormal 返回路径长度 != T (" + (path==null? "null": path.size()) + ")");
            }
            scenarios.add(new Scenario("S" + (s+1), 1.0 / numScenarios, path));
        }

        // 打印生成的场景（便于调试）
        System.out.println("生成场景数: " + scenarios.size());
        for(Scenario s : scenarios){
            System.out.println("场景 " + s.id + " (prob=" + s.prob + ")");
            for(int t=0; t<T; t++){
                System.out.println("  期 " + (t+1) + " 需求: " + s.demands.get(t).toString());
            }
        }
    }
    
    /**
     * 设置外部提供的场景数据
     */
    public void setScenarios(ArrayList<ArrayList<HashMap<Integer,Integer>>> scenarioData) {
        scenarios.clear();
        double prob = 1.0 / scenarioData.size();
        for(int i = 0; i < scenarioData.size(); i++) {
            scenarios.add(new Scenario("S" + (i+1), prob, scenarioData.get(i)));
        }
        
        // 打印设置的场景（便于调试）
        System.out.println("设置场景数: " + scenarios.size());
        for(Scenario s : scenarios){
            System.out.println("场景 " + s.id + " (prob=" + s.prob + ")");
            for(int t=0; t<T; t++){
                System.out.println("  期 " + (t+1) + " 需求: " + s.demands.get(t).toString());
            }
        }
    }

    /* -------------------- 构建并求解 MILP -------------------- */

    public void buildAndSolve(double timeLimitSeconds) throws Exception {
        cplex = new IloCplex();
        cplex.setParam(IloCplex.Param.TimeLimit, timeLimitSeconds);

        Y = new HashMap<>();
        I = new HashMap<>();
        B = new HashMap<>();
        Q = new HashMap<>();
        R_I = new HashMap<>();

        // 1) 第一阶段采购变量 Y（所有原材料）
        for(Material m : materials){
            IloNumVar v = cplex.numVar(0.0, Double.MAX_VALUE, IloNumVarType.Float, "Y_" + m.type);
            Y.put(m.type, v);
        }

        // 2) 对每个场景构造各类变量：成品 I/B，原料库存 R_I，切割方案 Q
        for(Scenario s : scenarios){
            Map<Integer, IloNumVar[]> imap = new HashMap<>();
            Map<Integer, IloNumVar[]> bmap = new HashMap<>();
            Map<String, IloNumVar[]> rmap = new HashMap<>();
            Map<String, List<IloNumVar[]>> qmap = new HashMap<>();

            // 成品库存/缺货
            for(Block b : blocks){
                IloNumVar[] Iarr = new IloNumVar[T+1];
                IloNumVar[] Barr = new IloNumVar[T+1];
                for(int t=1; t<=T; t++){
                    Iarr[t] = cplex.numVar(0.0, Double.MAX_VALUE, IloNumVarType.Float, "I_" + s.id + "_p" + b.index + "_t" + t);
                    Barr[t] = cplex.numVar(0.0, Double.MAX_VALUE, IloNumVarType.Float, "B_" + s.id + "_p" + b.index + "_t" + t);
                }
                imap.put(b.index, Iarr);
                bmap.put(b.index, Barr);
            }

            // 原材料库存
            for(Material m : materials){
                IloNumVar[] Rarr = new IloNumVar[T+1];
                for(int t=1; t<=T; t++){
                    Rarr[t] = cplex.numVar(0.0, Double.MAX_VALUE, IloNumVarType.Float, "IRaw_" + s.id + "_" + m.type + "_t" + t);
                }
                rmap.put(m.type, Rarr);
            }

            // 切割方案：先生成 patterns（缓存到 patternsByRaw）
            for(Material m : materials){
                ArrayList<Pattern> pats;
                if(patternsByRaw.containsKey(m.type)){
                    pats = patternsByRaw.get(m.type);
                } else {
                    pats = patternsGenerate(m, blocks);
                    patternsByRaw.put(m.type, pats);
                }
                // 打印该原材料的 pattern（便于调试）
                System.out.println("原材料 " + m.type + " 生成 " + pats.size() + " 个 pattern:");
                for(int pi=0; pi<pats.size(); pi++){
                    System.out.println("  pat " + pi + " : scheme = " + pats.get(pi).scheme.toString());
                }

                List<IloNumVar[]> patVars = new ArrayList<>();
                for(int pi=0; pi<pats.size(); pi++){
                    IloNumVar[] arr = new IloNumVar[T+1];
                    for(int t=1; t<=T; t++){
                        arr[t] = cplex.numVar(0.0, Double.MAX_VALUE, IloNumVarType.Float,
                                "Q_" + s.id + "_" + m.type + "_pat" + pi + "_t" + t);
                    }
                    patVars.add(arr);
                }
                qmap.put(m.type, patVars);
            }

            I.put(s.id, imap);
            B.put(s.id, bmap);
            R_I.put(s.id, rmap);
            Q.put(s.id, qmap);
        }

        // 3) 构造目标函数：采购成本 + 场景期望的库存/缺货/原料库存成本
        IloLinearNumExpr obj = cplex.linearNumExpr();
        for(Material m : materials){
            obj.addTerm(m.cost, Y.get(m.type));
        }
        // 场景期望成本
        for(Scenario s : scenarios){
            double p = s.prob;
            // 成品库存/缺货
            for(Block b : blocks){
                for(int t=1; t<=T; t++){
                    obj.addTerm(p * b.hold_cost, I.get(s.id).get(b.index)[t]);
                    obj.addTerm(p * b.back_cost, B.get(s.id).get(b.index)[t]);
                }
            }
            // 原材料库存成本
            for(Material m : materials){
                for(int t=1; t<=T; t++){
                    obj.addTerm(p * m.hold_cost, R_I.get(s.id).get(m.type)[t]);
                }
            }
        }
        cplex.addMinimize(obj);

        // 4) 约束：原材料平衡与成品平衡
        for(Scenario s : scenarios){
            // 原材料平衡
            for(Material m : materials){
                List<IloNumVar[]> patVars = Q.get(s.id).get(m.type);
                IloNumVar[] Iraw = R_I.get(s.id).get(m.type);
                // t = 1: Y_m - sum_p Q_mp1 - Iraw_1 = 0  -> addEq(expr,0)
                IloLinearNumExpr expr1 = cplex.linearNumExpr();
                expr1.addTerm(1.0, Y.get(m.type));
                for(IloNumVar[] arr : patVars) expr1.addTerm(-1.0, arr[1]);
                expr1.addTerm(-1.0, Iraw[1]);
                cplex.addEq(expr1, 0.0, "RawBal_" + s.id + "_" + m.type + "_t1");

                // t > 1: Iraw_{t-1} - sum_p Q_mpt - Iraw_t = 0
                for(int t=2; t<=T; t++){
                    IloLinearNumExpr expr = cplex.linearNumExpr();
                    expr.addTerm(1.0, Iraw[t-1]);
                    for(IloNumVar[] arr : patVars) expr.addTerm(-1.0, arr[t]);
                    expr.addTerm(-1.0, Iraw[t]);
                    cplex.addEq(expr, 0.0, "RawBal_" + s.id + "_" + m.type + "_t" + t);
                }
            }

            // 成品平衡
            for(Block b : blocks){
                for(int t=1; t<=T; t++){
                    IloLinearNumExpr expr = cplex.linearNumExpr();
                    if(t > 1){
                        expr.addTerm(1.0, I.get(s.id).get(b.index)[t-1]);
                        expr.addTerm(-1.0, B.get(s.id).get(b.index)[t-1]);
                    }
                    // 生产项：sum over raw types and patterns qty * Q
                    for(Material m : materials){
                        List<IloNumVar[]> patVars = Q.get(s.id).get(m.type);
                        ArrayList<Pattern> pats = patternsByRaw.get(m.type);
                        for(int pi=0; pi<pats.size(); pi++){
                            int qty = pats.get(pi).scheme.getOrDefault(b.index, 0);
                            if(qty > 0){
                                expr.addTerm(qty, patVars.get(pi)[t]);
                            }
                        }
                    }
                    // 将常数 demand 放在 RHS（cplex.addEq(expr, demand)）
                    int demand = s.demands.get(t-1).getOrDefault(b.index, 0);
                    expr.addTerm(-1.0, I.get(s.id).get(b.index)[t]);
                    expr.addTerm(1.0, B.get(s.id).get(b.index)[t]);
                    cplex.addEq(expr, (double)demand, "ProdBal_" + s.id + "_b" + b.index + "_t" + t);
                }
            }
        }

        // 可选：输出模型规模信息（变量/约束数量估计）
        System.out.println("模型构建完成，开始求解...");
        boolean solved = cplex.solve();
        if(solved){
            System.out.println("求解成功！目标值 = " + cplex.getObjValue());
            // 打印原材料采购量（非零）
            for(Material m : materials){
                double val = cplex.getValue(Y.get(m.type));
                System.out.printf("原材料 %s 采购量 = %.4f\n", m.type, val);
            }
            // 打印若干场景的关键结果（例如第一个场景的部分变量）
            Scenario s0 = scenarios.get(0);
            System.out.println("=== 场景 " + s0.id + " 的部分结果示例 ===");
            // 原材料库存
            for(Material m : materials){
                IloNumVar[] rarr = R_I.get(s0.id).get(m.type);
                for(int t=1; t<=T; t++){
                    double v = cplex.getValue(rarr[t]);
                    if(v > 1e-6) System.out.printf(" Iraw[%s]_t%d = %.4f\n", m.type, t, v);
                }
            }
            // 成品库存/缺货 & pattern 使用
            for(Block b : blocks){
                for(int t=1; t<=T; t++){
                    double iv = cplex.getValue(I.get(s0.id).get(b.index)[t]);
                    double bv = cplex.getValue(B.get(s0.id).get(b.index)[t]);
                    if(iv > 1e-6 || bv > 1e-6)
                        System.out.printf(" Prod %d t%d: I=%.4f, B=%.4f\n", b.index, t, iv, bv);
                }
            }
            // pattern usage (示例打印部分非零)
            for(Material m : materials){
                List<IloNumVar[]> plist = Q.get(s0.id).get(m.type);
                for(int pi=0; pi<plist.size(); pi++){
                    for(int t=1; t<=T; t++){
                        double qv = cplex.getValue(plist.get(pi)[t]);
                        if(qv > 1e-6){
                            System.out.printf(" Q[%s,pat%d,t%d] = %.4f\n", m.type, pi, t, qv);
                        }
                    }
                }
            }
        } else {
            System.out.println("模型求解失败，状态 = " + cplex.getStatus());
        }

        cplex.end();
    }

    public static void main(String[] args) {
        String materialFile = "C:\\Users\\Rain\\Desktop\\graduation24\\src\\data\\materials_demo.csv";
        String blockFile = "C:\\Users\\Rain\\Desktop\\graduation24\\src\\data\\blocks_index.csv";
        int numScen = 500;
        double timeLimit = 10.0;

        MILPSolver app = new MILPSolver();
        try {
            app.loadMaterials(materialFile);
            app.loadBlocks(blockFile);

            // 构造 ScenarioTree（假设你的 ScenarioTree 有构造函数接受 blocks）
            ScenarioTree tree = new ScenarioTree(app.blocks);
            app.generateScenarios(numScen, tree);

            // 求解
            app.buildAndSolve(timeLimit);

        } catch(Exception ex){
            System.err.println("发生异常: " + ex.getMessage());
            ex.printStackTrace();
        }
    }
}
