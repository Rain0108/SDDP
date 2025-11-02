import ilog.concert.*;
import ilog.cplex.IloCplex;

import java.util.ArrayList;
import java.util.HashMap;

public class CplexModel {
    public ArrayList<Material> rawMaterials;  //原材料集合
    public ArrayList<Block> blocks;  //零件集合
    public ArrayList<ArrayList<Pattern>> allPatterns;  //所有切割方案
    public int[][] D_it;  //成品管材i第t期的需求量
    public int[][][] a_ijk; //原料i使用自己的方案j切出的零件k数量
    public int T; //总周期数
    public IloCplex[][] models;
    public IloCplex[][] models_IP;
    public String[][][] cuts;
    public IloNumVar[] Y_i;
    public IloIntVar[] Y_i_IP;
    public IloNumVar[][][] Ic_it;
    public IloIntVar[][][] Ic_it_IP;
    public IloNumVar[][][] Ie_it;  //这是t+1周期的变量  [0,T]
    public IloIntVar[][][] Ie_it_IP;
    public IloNumVar[][][] B_it;   //这是t+1周期的变量  [0,T]
    public IloIntVar[][][] B_it_IP;
    public IloNumVar[][][][] Q_ijt;//[0,T-1]
    public IloIntVar[][][][] Q_ijt_IP;
    public IloNumVar[][] F;  //[0,T-1]
    public IloIntVar[][] F_IP;
    public IloRange[][][] blockStockRanges;  //第n个节点在第t周期的第i个成品库存约束的返回值，用于获取对偶变量  [0, T]
    public IloRange[][][] blockStockRanges_IP;
    public IloRange[][][] materialProdRanges;  //第n个节点在第t周期的第i个原材料生产约束的返回值，用于获取对偶变量  [0, T-1]
    public IloRange[][][] materialProdRanges_IP;
    public IloRange[][][] demandUBRanges;  //第n个节点在第t周期的第i个成品需求上限约束的返回值，用于获取对偶变量  [0, T-1]
    public IloRange[][][] demandUBRanges_IP;
    public IloRange[][][] cutRanges;  //第n个节点在第t周期的第i个benders cut的返回值  [0, T-1]
    public IloRange[][][] cutRanges_IP;
    public double[][][] RHS_Const;
    ScenarioTree scenarioTree;
    public IloCplex forwardPass_Stage1() throws Exception {
        //前向传播第一阶段模型建立
        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TimeLimit, Parameters.timeLimit);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        //原料i的备货量
        for(int i=0;i<Y_i.length;i++){
            Y_i[i] = cplex.numVar(0, Double.MAX_VALUE, "Y"+(i+1));
        }
        for(int i=0;i<Ic_it[0].length;i++){
            Ic_it[0][i][0] = cplex.numVar(0, Double.MAX_VALUE, "Ic1,"+(i+1)+","+"1");
        }
        for(int i=0;i<Q_ijt[0].length;i++){
            for(int j=0;j<Q_ijt[0][i].length;j++){
                Q_ijt[0][i][j][0] = cplex.numVar(0, Integer.MAX_VALUE, "Q1,"+(i+1)+","+(j+1)+",1");
            }
        }
        for(int i=0;i<B_it[0].length;i++){
            B_it[0][i][0] = cplex.numVar(0, Double.MAX_VALUE, "B"+"1,"+(i+1)+",0");
        }
        for(int i=0;i<Ie_it[0].length;i++){
            Ie_it[0][i][0] = cplex.numVar(0, Double.MAX_VALUE, "Ie"+"1,"+(i+1)+",0");
        }
        //第一阶段开始
        F[0][0] = cplex.numVar(0, Double.MAX_VALUE, "F2");
        IloNumExpr obj = cplex.numExpr();
        obj = cplex.sum(obj, F[0][0]);
        for(int i=0;i<rawMaterials.size();i++){
            IloNumExpr obj1 = cplex.prod(rawMaterials.get(i).getCost(), Y_i[i]);
            obj1 = cplex.sum(obj1, cplex.prod(rawMaterials.get(i).getHold_cost(), Ic_it[0][i][0]));
            for(int j=0;j<allPatterns.get(i).size();j++){
                obj1 = cplex.sum(obj1, cplex.prod(Q_ijt[0][i][j][0], Parameters.cutCost));
            }
            obj = cplex.sum(obj, obj1);
        }
        for(int i=0;i< blocks.size();i++){
            IloNumExpr obj1 = cplex.prod(blocks.get(i).getHold_cost(), Ie_it[0][i][0]);
            obj1 = cplex.sum(obj1, cplex.prod(blocks.get(i).getBack_cost(), B_it[0][i][0]));
            obj = cplex.sum(obj, obj1);
        }//添加第0周期的成品变量
        cplex.addMinimize(obj);
        for(int i=0;i<blocks.size();i++){
            IloNumExpr c1 = cplex.numExpr();
            c1 = cplex.sum(c1, cplex.diff(B_it[0][i][0], Ie_it[0][i][0]));
            blockStockRanges[0][0][i] = cplex.addEq(c1, 0, "blockStock"+"0,"+"0,"+(i+1));
        }
        for(int i=0;i<rawMaterials.size();i++){
            IloNumExpr c1 = cplex.diff(Ic_it[0][i][0], Y_i[i]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                c1 = cplex.sum(c1, Q_ijt[0][i][j][0]);
            }
            materialProdRanges[0][0][i] = cplex.addEq(c1, 0, "materialProd"+"1,1,"+(i+1));
        }
        ArrayList<Integer> worstDemand = scenarioTree.getWorstPath();
        for(int k=0;k<blocks.size();k++){
            IloNumExpr c2 = cplex.numExpr();
            for(int i=0;i<rawMaterials.size();i++){
                for(int j=0;j<allPatterns.get(i).size();j++){
                    c2 = cplex.sum(c2, cplex.prod(a_ijk[i][j][k], Q_ijt[0][i][j][0]));
                }
            }
            demandUBRanges[0][0][k] = cplex.addLe(c2, worstDemand.get(k), "demandUB1,1,"+(k+1));
        }
        return cplex;
    }
    public IloCplex forwardPass_Stage1_IP() throws Exception {
        //前向传播第一阶段整数模型建立
        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TimeLimit, Parameters.timeLimit);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        //原料i的备货量
        for(int i=0;i<Y_i_IP.length;i++){
            Y_i_IP[i] = cplex.intVar(0, Integer.MAX_VALUE, "Y"+(i+1)+",IP");
        }
        for(int i=0;i<Ic_it_IP[0].length;i++){
            Ic_it_IP[0][i][0] = cplex.intVar(0, Integer.MAX_VALUE, "Ic1,"+(i+1)+","+"1"+",IP");
        }
        for(int i=0;i<Q_ijt_IP[0].length;i++){
            for(int j=0;j<Q_ijt_IP[0][i].length;j++){
                Q_ijt_IP[0][i][j][0] = cplex.intVar(0, Integer.MAX_VALUE, "Q1,"+(i+1)+","+(j+1)+",1"+",IP");
            }
        }
        for(int i=0;i<B_it_IP[0].length;i++){
            B_it_IP[0][i][0] = cplex.intVar(0, Integer.MAX_VALUE, "B"+"1,"+(i+1)+",0"+",IP");
        }
        for(int i=0;i<Ie_it_IP[0].length;i++){
            Ie_it_IP[0][i][0] = cplex.intVar(0, Integer.MAX_VALUE, "Ie"+"1,"+(i+1)+",0"+",IP");
        }
        //第一阶段开始
        F_IP[0][0] = cplex.intVar(0, Integer.MAX_VALUE, "F2"+",IP");
        IloNumExpr obj = cplex.numExpr();
        obj = cplex.sum(obj, F_IP[0][0]);
        for(int i=0;i<rawMaterials.size();i++){
            IloNumExpr obj1 = cplex.prod(rawMaterials.get(i).getCost(), Y_i_IP[i]);
            obj1 = cplex.sum(obj1, cplex.prod(rawMaterials.get(i).getHold_cost(), Ic_it_IP[0][i][0]));
            for(int j=0;j<allPatterns.get(i).size();j++){
                obj1 = cplex.sum(obj1, cplex.prod(Q_ijt_IP[0][i][j][0], Parameters.cutCost));
            }
            obj = cplex.sum(obj, obj1);
        }
        for(int i=0;i< blocks.size();i++){
            IloNumExpr obj1 = cplex.prod(blocks.get(i).getHold_cost(), Ie_it_IP[0][i][0]);
            obj1 = cplex.sum(obj1, cplex.prod(blocks.get(i).getBack_cost(), B_it_IP[0][i][0]));
            obj = cplex.sum(obj, obj1);
        }//添加第0周期的成品变量
        cplex.addMinimize(obj);
        for(int i=0;i<blocks.size();i++){
            IloNumExpr c1 = cplex.numExpr();
            c1 = cplex.sum(c1, cplex.diff(B_it_IP[0][i][0], Ie_it_IP[0][i][0]));
            blockStockRanges_IP[0][0][i] = cplex.addEq(c1, 0, "blockStock"+"0,"+"0,"+(i+1)+",IP");
        }
        for(int i=0;i<rawMaterials.size();i++){
            IloNumExpr c1 = cplex.diff(Ic_it_IP[0][i][0], Y_i_IP[i]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                c1 = cplex.sum(c1, Q_ijt_IP[0][i][j][0]);
            }
            materialProdRanges_IP[0][0][i] = cplex.addEq(c1, 0, "materialProd"+"1,1,"+(i+1)+",IP");
        }
        ArrayList<Integer> worstDemand = scenarioTree.getWorstPath();
        for(int k=0;k<blocks.size();k++){
            IloNumExpr c2 = cplex.numExpr();
            for(int i=0;i<rawMaterials.size();i++){
                for(int j=0;j<allPatterns.get(i).size();j++){
                    c2 = cplex.sum(c2, cplex.prod(a_ijk[i][j][k], Q_ijt_IP[0][i][j][0]));
                }
            }
            demandUBRanges_IP[0][0][k] = cplex.addLe(c2, worstDemand.get(k), "demandUB1,1,"+(k+1)+",IP");
        }
        return cplex;
    }
    public IloCplex forwardPass_Stage2toT(int t, Solution solution, int curNode) throws Exception{
        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TimeLimit, Parameters.timeLimit);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        //建立第t阶段模型
        for(int i=0;i<Ic_it[curNode].length;i++){
            Ic_it[curNode][i][t] = cplex.numVar(0, Double.MAX_VALUE, "Ic"+(curNode+1)+","+(i+1)+","+(t+1));
        }
        for(int i=0;i<Q_ijt[curNode].length;i++){
            for(int j=0;j<Q_ijt[curNode][i].length;j++){
                Q_ijt[curNode][i][j][t] = cplex.numVar(0, Double.MAX_VALUE, "Q"+(curNode+1)+","+(i+1)+","+(j+1)+","+(t+1));
            }
        }
        for(int i=0;i<B_it[curNode].length;i++){
            B_it[curNode][i][t] = cplex.numVar(0, Double.MAX_VALUE, "B"+(curNode+1)+","+(i+1)+","+t);
        }
        for(int i=0;i<Ie_it[curNode].length;i++){
            Ie_it[curNode][i][t] = cplex.numVar(0, Double.MAX_VALUE, "Ie"+(curNode+1)+","+(i+1)+","+t);
        }
        F[curNode][t] = cplex.numVar(0, Double.MAX_VALUE, "F"+(t+2));  //F是对从t+1开始的后续周期成本估计
        IloNumExpr obj = cplex.numExpr();
        obj = cplex.sum(obj, F[curNode][t]);
        for(int i=0;i<rawMaterials.size();i++){
            obj = cplex.sum(obj, cplex.prod(rawMaterials.get(i).getHold_cost(), Ic_it[curNode][i][t]));
            for(int j=0;j<allPatterns.get(i).size();j++){
                obj = cplex.sum(obj, cplex.prod(Q_ijt[curNode][i][j][t], Parameters.cutCost));
            }
        }
        for(int i=0;i< blocks.size();i++){
            IloNumExpr obj1 = cplex.prod(blocks.get(i).getHold_cost(), Ie_it[curNode][i][t]);
            obj1 = cplex.sum(obj1, cplex.prod(blocks.get(i).getBack_cost(), B_it[curNode][i][t]));
            obj = cplex.sum(obj, obj1);
        }
        cplex.addMinimize(obj);
        //约束1：t-1周期末的库存与缺货结算
        for(int i=0;i<blocks.size();i++){
            IloNumExpr c1 = cplex.numExpr();
            double tempV = 0;
            if(t > 1) {
                tempV += (solution.Ie_value[solution.path.get(t-2)][i][t-1] - solution.B_value[solution.path.get(t-2)][i][t-1]);
            }
            for (int k = 0; k < rawMaterials.size(); k++) {
                for (int j = 0; j < allPatterns.get(k).size(); j++) {
                    tempV += a_ijk[k][j][i] * solution.Q_value[t-2-Parameters.leadTime>=0?solution.path.get(t-2-Parameters.leadTime):0][k][j][t-1-Parameters.leadTime];
                }
            }
            c1 = cplex.sum(c1, tempV);
            c1 = cplex.diff(c1, scenarioTree.tree.get(t-1).getCurLayer().get(curNode).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it[curNode][i][t], Ie_it[curNode][i][t]));
            blockStockRanges[curNode][t][i] = cplex.addEq(c1, 0, "blockStock"+(curNode+1)+","+t+","+(i+1));
        }
        //约束2：t周期的生产
        for(int i=0;i<rawMaterials.size();i++){
            IloNumExpr c2 = cplex.diff(solution.Ic_value[t>1?solution.path.get(t-2):0][i][t-1], Ic_it[curNode][i][t]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                c2 = cplex.diff(c2, Q_ijt[curNode][i][j][t]);
            }
            materialProdRanges[curNode][t][i] = cplex.addEq(c2, 0, "materialProd"+(curNode+1)+","+(t+1)+","+(i+1));
        }
        ArrayList<Integer> worstDemand = scenarioTree.getWorstPath();
        for(int k=0;k<blocks.size();k++){
            IloNumExpr c3 = cplex.numExpr();
            for(int i=0;i<rawMaterials.size();i++){
                for(int j=0;j<allPatterns.get(i).size();j++){
                    c3 = cplex.sum(c3, cplex.prod(a_ijk[i][j][k], Q_ijt[curNode][i][j][t]));
                }
            }
            demandUBRanges[curNode][t][k] = cplex.addLe(c3, worstDemand.get(k), "demandUB"+(curNode+1)+","+(t+1)+","+(k+1));
        }
        return cplex;
    }
    public IloCplex forwardPass_Stage2toT_IP(int t, Solution solution, int curNode) throws Exception{
        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TimeLimit, Parameters.timeLimit);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        //建立第t阶段模型
        for(int i=0;i<Ic_it_IP[curNode].length;i++){
            Ic_it_IP[curNode][i][t] = cplex.intVar(0, Integer.MAX_VALUE, "Ic"+(curNode+1)+","+(i+1)+","+(t+1)+",IP");
        }
        for(int i=0;i<Q_ijt_IP[curNode].length;i++){
            for(int j=0;j<Q_ijt_IP[curNode][i].length;j++){
                Q_ijt_IP[curNode][i][j][t] = cplex.intVar(0, Integer.MAX_VALUE, "Q"+(curNode+1)+","+(i+1)+","+(j+1)+","+(t+1)+",IP");
            }
        }
        for(int i=0;i<B_it_IP[curNode].length;i++){
            B_it_IP[curNode][i][t] = cplex.intVar(0, Integer.MAX_VALUE, "B"+(curNode+1)+","+(i+1)+","+t+",IP");
        }
        for(int i=0;i<Ie_it_IP[curNode].length;i++){
            Ie_it_IP[curNode][i][t] = cplex.intVar(0, Integer.MAX_VALUE, "Ie"+(curNode+1)+","+(i+1)+","+t+",IP");
        }
        F_IP[curNode][t] = cplex.intVar(0, Integer.MAX_VALUE, "F"+(t+2)+",IP");  //F是对从t+1开始的后续周期成本估计
        IloNumExpr obj = cplex.numExpr();
        obj = cplex.sum(obj, F_IP[curNode][t]);
        for(int i=0;i<rawMaterials.size();i++){
            obj = cplex.sum(obj, cplex.prod(rawMaterials.get(i).getHold_cost(), Ic_it_IP[curNode][i][t]));
            for(int j=0;j<allPatterns.get(i).size();j++){
                obj = cplex.sum(obj, cplex.prod(Q_ijt_IP[curNode][i][j][t], Parameters.cutCost));
            }
        }
        for(int i=0;i< blocks.size();i++){
            IloNumExpr obj1 = cplex.prod(blocks.get(i).getHold_cost(), Ie_it_IP[curNode][i][t]);
            obj1 = cplex.sum(obj1, cplex.prod(blocks.get(i).getBack_cost(), B_it_IP[curNode][i][t]));
            obj = cplex.sum(obj, obj1);
        }
        cplex.addMinimize(obj);
        //约束1：t-1周期末的库存与缺货结算
        for(int i=0;i<blocks.size();i++){
            IloNumExpr c1 = cplex.numExpr();
            double tempV = 0;
            if(t > 1) {
                tempV += (solution.Ie_value_IP[solution.path.get(t-2)][i][t-1] - solution.B_value_IP[solution.path.get(t-2)][i][t-1]);
            }
            for (int k = 0; k < rawMaterials.size(); k++) {
                for (int j = 0; j < allPatterns.get(k).size(); j++) {
                    tempV += a_ijk[k][j][i] * solution.Q_value_IP[t-2-Parameters.leadTime>=0?solution.path.get(t-2-Parameters.leadTime):0][k][j][t-1-Parameters.leadTime];
                }
            }
            c1 = cplex.sum(c1, tempV);
            c1 = cplex.diff(c1, scenarioTree.tree.get(t-1).getCurLayer().get(curNode).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it_IP[curNode][i][t], Ie_it_IP[curNode][i][t]));
            blockStockRanges_IP[curNode][t][i] = cplex.addEq(c1, 0, "blockStock"+(curNode+1)+","+t+","+(i+1)+",IP");
        }
        //约束2：t周期的生产
        for(int i=0;i<rawMaterials.size();i++){
            IloNumExpr c2 = cplex.diff(solution.Ic_value_IP[t>1?solution.path.get(t-2):0][i][t-1], Ic_it_IP[curNode][i][t]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                c2 = cplex.diff(c2, Q_ijt_IP[curNode][i][j][t]);
            }
            materialProdRanges_IP[curNode][t][i] = cplex.addEq(c2, 0, "materialProd"+(curNode+1)+","+(t+1)+","+(i+1)+",IP");
        }
        ArrayList<Integer> worstDemand = scenarioTree.getWorstPath();
        for(int k=0;k<blocks.size();k++){
            IloNumExpr c3 = cplex.numExpr();
            for(int i=0;i<rawMaterials.size();i++){
                for(int j=0;j<allPatterns.get(i).size();j++){
                    c3 = cplex.sum(c3, cplex.prod(a_ijk[i][j][k], Q_ijt_IP[curNode][i][j][t]));
                }
            }
            demandUBRanges_IP[curNode][t][k] = cplex.addLe(c3, worstDemand.get(k), "demandUB"+(curNode+1)+","+(t+1)+","+(k+1)+",IP");
        }
        return cplex;
    }
    public IloCplex forwardPass_StageT_1(Solution solution, int curNode) throws Exception{
        //前向传播第（T+1）阶段
        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TimeLimit, Parameters.timeLimit);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        //建立第T+1阶段模型
        for(int i=0;i<Ie_it[curNode].length;i++){
            Ie_it[curNode][i][T] = cplex.numVar(0, Double.MAX_VALUE, "Ie"+(curNode+1)+","+(i+1)+",T+1");
        }
        for(int i=0;i<B_it[curNode].length;i++){
            B_it[curNode][i][T] = cplex.numVar(0, Double.MAX_VALUE, "B"+(curNode+1)+","+(i+1)+",T+1");
        }
        IloNumExpr obj = cplex.numExpr();
        for(int i=0;i< blocks.size();i++){
            IloNumExpr obj1 = cplex.prod(blocks.get(i).getHold_cost(), Ie_it[curNode][i][T]);
            obj1 = cplex.sum(obj1, cplex.prod(blocks.get(i).getBack_cost(), B_it[curNode][i][T]));
            obj = cplex.sum(obj, obj1);
        }
        cplex.addMinimize(obj);
        //约束1：T周期末的库存与缺货结算
        for(int i=0;i<blocks.size();i++){
            IloNumExpr c1 = cplex.numExpr();
            if(T > 1) {
                c1 = cplex.sum(c1, solution.Ie_value[solution.path.get(T-2)][i][T-1] - solution.B_value[solution.path.get(T-2)][i][T-1]);
            }
            for(int k=0;k< rawMaterials.size();k++){
                for(int j=0;j<allPatterns.get(k).size();j++){
                    c1 = cplex.sum(c1, a_ijk[k][j][i] * solution.Q_value[T-2-Parameters.leadTime>=0?(solution.path.get(T-2-Parameters.leadTime)):0][k][j][T-1-Parameters.leadTime]);
                }
            }
            c1 = cplex.diff(c1, scenarioTree.tree.get(T-1).getCurLayer().get(curNode).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it[curNode][i][T], Ie_it[curNode][i][T]));
            blockStockRanges[curNode][T][i] = cplex.addEq(c1, 0, "blockStock"+(curNode+1)+","+T+","+(i+1));
        }
        return cplex;
    }
    public IloCplex forwardPass_StageT_1_IP(Solution solution, int curNode) throws Exception{
        //前向传播第（T+1）阶段
        IloCplex cplex = new IloCplex();
        cplex.setParam(IloCplex.DoubleParam.TimeLimit, Parameters.timeLimit);
        cplex.setParam(IloCplex.Param.MIP.Tolerances.MIPGap, 0.01);
        //建立第T+1阶段模型
        for(int i=0;i<Ie_it_IP[curNode].length;i++){
            Ie_it_IP[curNode][i][T] = cplex.intVar(0, Integer.MAX_VALUE, "Ie"+(curNode+1)+","+(i+1)+",T+1"+",IP");
        }
        for(int i=0;i<B_it_IP[curNode].length;i++){
            B_it_IP[curNode][i][T] = cplex.intVar(0, Integer.MAX_VALUE, "B"+(curNode+1)+","+(i+1)+",T+1"+",IP");
        }
        IloNumExpr obj = cplex.numExpr();
        for(int i=0;i< blocks.size();i++){
            IloNumExpr obj1 = cplex.prod(blocks.get(i).getHold_cost(), Ie_it_IP[curNode][i][T]);
            obj1 = cplex.sum(obj1, cplex.prod(blocks.get(i).getBack_cost(), B_it_IP[curNode][i][T]));
            obj = cplex.sum(obj, obj1);
        }
        cplex.addMinimize(obj);
        //约束1：T周期末的库存与缺货结算
        for(int i=0;i<blocks.size();i++){
            IloNumExpr c1 = cplex.numExpr();
            if(T > 1) {
                c1 = cplex.sum(c1, solution.Ie_value_IP[solution.path.get(T-2)][i][T-1] - solution.B_value_IP[solution.path.get(T-2)][i][T-1]);
            }
            for(int k=0;k< rawMaterials.size();k++){
                for(int j=0;j<allPatterns.get(k).size();j++){
                    c1 = cplex.sum(c1, a_ijk[k][j][i] * solution.Q_value_IP[T-2-Parameters.leadTime>=0?(solution.path.get(T-2-Parameters.leadTime)):0][k][j][T-1-Parameters.leadTime]);
                }
            }
            c1 = cplex.diff(c1, scenarioTree.tree.get(T-1).getCurLayer().get(curNode).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it_IP[curNode][i][T], Ie_it_IP[curNode][i][T]));
            blockStockRanges_IP[curNode][T][i] = cplex.addEq(c1, 0, "blockStock"+(curNode+1)+","+T+","+(i+1)+",IP");
        }
        return cplex;
    }
    public Solution forwardPass_Setup(int T, ArrayList<Integer> path) throws Exception{
        //首先求解第一阶段
        System.out.println("开始建立第一阶段松弛模型");
        models[0][0] = forwardPass_Stage1();  //建立起第一阶段模型，对应models[0]
        System.out.println("开始建立第一阶段整数模型");
        models_IP[0][0] = forwardPass_Stage1_IP();
        IloCplex cplex = models_IP[0][0];
        System.out.println("开始第一阶段求解");
        boolean isSolved = cplex.solve();
        if(!isSolved) throw new Exception("第一阶段求解失败");
        Solution solution = new Solution(rawMaterials, blocks, allPatterns, T, path);
        solution.obj_value_IP[0][0] = cplex.getObjValue();
        solution.Y_value_IP = cplex.getValues(Y_i_IP);
        solution.F_value_IP[0][0] = cplex.getValue(F_IP[0][0]);
        for(int i=0;i<solution.Ic_value_IP[0].length;i++){
            solution.Ic_value_IP[0][i][0] = cplex.getValue(Ic_it_IP[0][i][0]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                solution.Q_value_IP[0][i][j][0] = cplex.getValue(Q_ijt_IP[0][i][j][0]);
            }
        }
        for(int i=0;i<blocks.size();i++){
            solution.Ie_value_IP[0][i][0] = cplex.getValue(Ie_it_IP[0][i][0]);
            solution.B_value_IP[0][i][0] = cplex.getValue(B_it_IP[0][i][0]);
        }
        System.out.println("完成第一阶段求解");
        for(int t=1;t<T;t++){
            //这里把当前层的所有需求全部建模，但是只求解出现在场景路径上的那一个
            for(int l=0;l<Parameters.nodeNumPerLayer;l++) {
                models[l][t] = forwardPass_Stage2toT(t, solution, l);
                models_IP[l][t] = forwardPass_Stage2toT_IP(t, solution, l);
            }
            cplex = models_IP[path.get(t-1)][t];
            System.out.println("开始第"+(t+1)+"阶段求解");
            isSolved = cplex.solve();

            if(!isSolved) throw new Exception("第"+(t+1)+"阶段求解失败");
            solution.obj_value_IP[path.get(t-1)][t] = cplex.getObjValue();
            solution.F_value_IP[path.get(t-1)][t] = cplex.getValue(F_IP[path.get(t-1)][t]);
            for(int i=0;i<solution.Ic_value_IP[path.get(t-1)].length;i++){
                solution.Ic_value_IP[path.get(t-1)][i][t] = cplex.getValue(Ic_it_IP[path.get(t-1)][i][t]);
                for(int j=0;j<allPatterns.get(i).size();j++){
                    solution.Q_value_IP[path.get(t-1)][i][j][t] = cplex.getValue(Q_ijt_IP[path.get(t-1)][i][j][t]);
                }
            }
            for(int i=0;i<blocks.size();i++){
                solution.Ie_value_IP[path.get(t-1)][i][t] = cplex.getValue(Ie_it_IP[path.get(t-1)][i][t]);
                solution.B_value_IP[path.get(t-1)][i][t] = cplex.getValue(B_it_IP[path.get(t-1)][i][t]);
            }
            //cplex.end();
            System.out.println("完成第"+(t+1)+"阶段求解");
        }
        //double var = models[path.get(T-2)][T-1].getValue(F[path.get(T-2)][2]);
        for(int l=0;l<Parameters.nodeNumPerLayer;l++) {
            models[l][T] = forwardPass_StageT_1(solution, l);
            models_IP[l][T] = forwardPass_StageT_1_IP(solution, l);
        }
        cplex = models_IP[path.get(T-1)][T];
        System.out.println("开始第(T+1)阶段求解");
        isSolved = cplex.solve();
        if(!isSolved) throw new Exception("第(T+1)阶段求解失败");
        solution.obj_value_IP[path.get(T-1)][T] = cplex.getObjValue();
        for(int i=0;i<blocks.size();i++){
            solution.Ie_value_IP[path.get(T-1)][i][T] = cplex.getValue(Ie_it_IP[path.get(T-1)][i][T]);
            solution.B_value_IP[path.get(T-1)][i][T] = cplex.getValue(B_it_IP[path.get(T-1)][i][T]);
        }
        //cplex.end();
        System.out.println("完成第(T+1)阶段求解");

        return solution;
    }
    public void modify_2toT(IloCplex cplex, int t, int n, Solution solution) throws Exception{
        for(int i=0;i<blocks.size();i++){
            cplex.remove(blockStockRanges[n][t][i]);
            IloNumExpr c1 = cplex.numExpr();
            double tempV = 0;
            if(t > 1) {
                tempV += (solution.Ie_value_IP[solution.path.get(t-2)][i][t-1] - solution.B_value_IP[solution.path.get(t-2)][i][t-1]);
            }
            for (int k = 0; k < rawMaterials.size(); k++) {
                for (int j = 0; j < allPatterns.get(k).size(); j++) {
                    tempV += a_ijk[k][j][i] * solution.Q_value_IP[t-2-Parameters.leadTime>=0?solution.path.get(t-2-Parameters.leadTime):0][k][j][t-1-Parameters.leadTime];
                }
            }
            c1 = cplex.sum(c1, tempV);
            c1 = cplex.diff(c1, scenarioTree.tree.get(t-1).getCurLayer().get(n).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it[n][i][t], Ie_it[n][i][t]));
            blockStockRanges[n][t][i] = cplex.addEq(c1, 0, "blockStock"+(n+1)+","+t+","+(i+1));
        }
        for(int i=0;i<rawMaterials.size();i++){
            cplex.remove(materialProdRanges[n][t][i]);
            IloNumExpr c2 = cplex.diff(solution.Ic_value_IP[t>1?solution.path.get(t-2):0][i][t-1], Ic_it[n][i][t]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                c2 = cplex.diff(c2, Q_ijt[n][i][j][t]);
            }
            materialProdRanges[n][t][i] = cplex.addEq(c2, 0, "materialProd"+(n+1)+","+(t+1)+","+(i+1));
        }
    }
    public void modify_2toT_IP(IloCplex cplex, int t, int n, Solution solution) throws Exception{
        for(int i=0;i<blocks.size();i++){
            cplex.remove(blockStockRanges_IP[n][t][i]);
            IloNumExpr c1 = cplex.numExpr();
            double tempV = 0;
            if(t > 1) {
                tempV += (solution.Ie_value_IP[solution.path.get(t-2)][i][t-1] - solution.B_value_IP[solution.path.get(t-2)][i][t-1]);
            }
            for (int k = 0; k < rawMaterials.size(); k++) {
                for (int j = 0; j < allPatterns.get(k).size(); j++) {
                    tempV += a_ijk[k][j][i] * solution.Q_value_IP[t-2-Parameters.leadTime>=0?solution.path.get(t-2-Parameters.leadTime):0][k][j][t-1-Parameters.leadTime];
                }
            }
            c1 = cplex.sum(c1, tempV);
            c1 = cplex.diff(c1, scenarioTree.tree.get(t-1).getCurLayer().get(n).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it_IP[n][i][t], Ie_it_IP[n][i][t]));
            blockStockRanges_IP[n][t][i] = cplex.addEq(c1, 0, "blockStock"+(n+1)+","+t+","+(i+1)+",IP");
        }
        for(int i=0;i<rawMaterials.size();i++){
            cplex.remove(materialProdRanges_IP[n][t][i]);
            IloNumExpr c2 = cplex.diff(solution.Ic_value_IP[t>1?solution.path.get(t-2):0][i][t-1], Ic_it_IP[n][i][t]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                c2 = cplex.diff(c2, Q_ijt_IP[n][i][j][t]);
            }
            materialProdRanges_IP[n][t][i] = cplex.addEq(c2, 0, "materialProd"+(n+1)+","+(t+1)+","+(i+1)+",IP");
        }
    }
    public void modify_T_1(IloCplex cplex, int curNode, Solution solution) throws Exception{
        for(int i=0;i<blocks.size();i++){
            cplex.remove(blockStockRanges[curNode][T][i]);
            IloNumExpr c1 = cplex.numExpr();
            if(T > 1) {
                c1 = cplex.sum(c1, solution.Ie_value_IP[solution.path.get(T-2)][i][T-1] - solution.B_value_IP[solution.path.get(T-2)][i][T-1]);
            }
            for(int k=0;k< rawMaterials.size();k++){
                for(int j=0;j<allPatterns.get(k).size();j++){
                    c1 = cplex.sum(c1, a_ijk[k][j][i] * solution.Q_value_IP[T-2-Parameters.leadTime>=0?(solution.path.get(T-2-Parameters.leadTime)):0][k][j][T-1-Parameters.leadTime]);
                }
            }
            c1 = cplex.diff(c1, scenarioTree.tree.get(T-1).getCurLayer().get(curNode).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it[curNode][i][T], Ie_it[curNode][i][T]));
            blockStockRanges[curNode][T][i] = cplex.addEq(c1, 0, "blockStock"+(curNode+1)+","+T+","+(i+1));
        }
    }
    public void modify_T_1_IP(IloCplex cplex, int curNode, Solution solution) throws Exception{
        for(int i=0;i<blocks.size();i++){
            cplex.remove(blockStockRanges_IP[curNode][T][i]);
            IloNumExpr c1 = cplex.numExpr();
            if(T > 1) {
                c1 = cplex.sum(c1, solution.Ie_value_IP[solution.path.get(T-2)][i][T-1] - solution.B_value_IP[solution.path.get(T-2)][i][T-1]);
            }
            for(int k=0;k< rawMaterials.size();k++){
                for(int j=0;j<allPatterns.get(k).size();j++){
                    c1 = cplex.sum(c1, a_ijk[k][j][i] * solution.Q_value_IP[T-2-Parameters.leadTime>=0?(solution.path.get(T-2-Parameters.leadTime)):0][k][j][T-1-Parameters.leadTime]);
                }
            }
            c1 = cplex.diff(c1, scenarioTree.tree.get(T-1).getCurLayer().get(curNode).get(blocks.get(i).getLength()));
            c1 = cplex.sum(c1, cplex.diff(B_it_IP[curNode][i][T], Ie_it_IP[curNode][i][T]));
            blockStockRanges_IP[curNode][T][i] = cplex.addEq(c1, 0, "blockStock"+(curNode+1)+","+T+","+(i+1)+",IP");
        }
    }
    public Solution forwardPass(int T, ArrayList<Integer> path) throws Exception{
        //首先求解第一阶段
        IloCplex cplex = models_IP[0][0];
        System.out.println("开始第1阶段求解");
        boolean isSolved = cplex.solve();
        if(!isSolved) throw new Exception("第1阶段求解失败");
        Solution solution = new Solution(rawMaterials, blocks, allPatterns, T, path);
        solution.obj_value_IP[0][0] = cplex.getObjValue();
        solution.F_value_IP[0][0] = cplex.getValue(F_IP[0][0]);
        solution.Y_value_IP = cplex.getValues(Y_i_IP);
        for(int i=0;i<solution.Ic_value_IP[0].length;i++){
            solution.Ic_value_IP[0][i][0] = cplex.getValue(Ic_it_IP[0][i][0]);
            for(int j=0;j<allPatterns.get(i).size();j++){
                solution.Q_value_IP[0][i][j][0] = cplex.getValue(Q_ijt_IP[0][i][j][0]);
            }
        }
        for(int i=0;i<blocks.size();i++){
            solution.Ie_value_IP[0][i][0] = cplex.getValue(Ie_it_IP[0][i][0]);
            solution.B_value_IP[0][i][0] = cplex.getValue(B_it_IP[0][i][0]);
        }
        System.out.println("完成第1阶段求解");
        for(int t=1;t<T;t++){
            //这里求解出现在场景路径上的那一个
            for(int l=0;l<Parameters.nodeNumPerLayer;l++) {
                modify_2toT(models[l][t], t, l ,solution);
                modify_2toT_IP(models_IP[l][t], t, l, solution);
            }
            cplex = models_IP[path.get(t-1)][t];
            System.out.println("开始第"+(t+1)+"阶段求解");
            isSolved = cplex.solve();
            if(!isSolved) throw new Exception("第"+(t+1)+"阶段求解失败");
            solution.obj_value_IP[path.get(t-1)][t] = cplex.getObjValue();
            solution.F_value_IP[path.get(t-1)][t] = cplex.getValue(F_IP[path.get(t-1)][t]);
            for(int i=0;i<solution.Ic_value_IP[path.get(t-1)].length;i++){
                solution.Ic_value_IP[path.get(t-1)][i][t] = cplex.getValue(Ic_it_IP[path.get(t-1)][i][t]);
                for(int j=0;j<allPatterns.get(i).size();j++){
                    solution.Q_value_IP[path.get(t-1)][i][j][t] = cplex.getValue(Q_ijt_IP[path.get(t-1)][i][j][t]);
                }
            }
            for(int i=0;i<blocks.size();i++){
                solution.Ie_value_IP[path.get(t-1)][i][t] = cplex.getValue(Ie_it_IP[path.get(t-1)][i][t]);
                solution.B_value_IP[path.get(t-1)][i][t] = cplex.getValue(B_it_IP[path.get(t-1)][i][t]);
            }
            //cplex.end();
            System.out.println("完成第"+(t+1)+"阶段求解");
        }
        for(int l=0;l<Parameters.nodeNumPerLayer;l++) {
            modify_T_1(models[l][T], l, solution);
            modify_T_1_IP(models_IP[l][T], l, solution);
        }
        cplex = models_IP[path.get(T-1)][T];
        System.out.println("开始第(T+1)阶段求解");
        isSolved = cplex.solve();
        if(!isSolved) throw new Exception("第(T+1)阶段求解失败");
        solution.obj_value_IP[path.get(T-1)][T] = cplex.getObjValue();
        for(int i=0;i<blocks.size();i++){
            solution.Ie_value_IP[path.get(T-1)][i][T] = cplex.getValue(Ie_it_IP[path.get(T-1)][i][T]);
            solution.B_value_IP[path.get(T-1)][i][T] = cplex.getValue(B_it_IP[path.get(T-1)][i][T]);
        }
        //cplex.end();
        System.out.println("完成第(T+1)阶段求解");
        return solution;
    }
    public void backwardPass(ArrayList<Integer> path, Solution solution, int iter) throws Exception {
        //反向传播
        System.out.println("---------------------开始反向传播-------------------");
        ArrayList<Integer> worstDemand = scenarioTree.getWorstPath();
        //T+1周期向T周期模型添加约束
        double[][] duals = new double[Parameters.nodeNumPerLayer][];
        //首先获取路径上已求解的T+1周期模型对偶变量（仅包含成品库存约束）
        //duals[path.get(T-1)] = models[path.get(T-1)][T].getDuals(blockStockRanges[path.get(T-1)][T]);
        //建立T+1周期其他节点的模型，求解并获取对偶变量
        ModelSolver solver = new ModelSolver(T, models);
        solver.solve();
        for(int i=0;i<Parameters.nodeNumPerLayer;i++){
            //if(!models[i][T].solve()) throw new Exception("反向传播模型求解失败");
            //System.out.println(models[i][T].getStatus());
            duals[i] = models[i][T].getDuals(blockStockRanges[i][T]);
        }
        //向T周期所有节点模型添加约束
        for(int n=0;n<Parameters.nodeNumPerLayer;n++) {
            //cut要添加到T周期第n个节点模型中
            IloCplex cplex = models[n][T - 1];
            IloCplex cplex_IP = models_IP[n][T-1];
            if(cplex.solve()) System.out.println("T周期添加benders前"+cplex.getObjValue());
            System.out.println(cplex.getValue(F[n][T-1]));
            IloNumExpr exp = cplex.numExpr();
            exp = cplex.sum(exp, F[n][T - 1]);
            IloNumExpr exp_IP = cplex_IP.numExpr();
            exp_IP = cplex.sum(exp_IP, F_IP[n][T - 1]);
            IloNumExpr rhs = cplex.numExpr();
            IloNumExpr rhs_IP = cplex_IP.numExpr();
            double cons = 0;
            for (int d = 0; d < Parameters.nodeNumPerLayer; d++) {
                IloNumExpr Fl = cplex.numExpr();
                IloNumExpr Fl_IP = cplex_IP.numExpr();
                double tempC1 = 0;
                for (int i = 0; i < blocks.size(); i++) {
                    IloNumExpr tempF = cplex.numExpr();
                    IloNumExpr tempF_IP = cplex_IP.numExpr();
                    double tempC = 0;
                    tempF = cplex.diff(tempF, Ie_it[n][i][T - 1]);
                    tempF = cplex.sum(tempF, B_it[n][i][T - 1]);
                    tempF = cplex.sum(tempF, scenarioTree.tree.get(T - 1).getCurLayer().get(d).get(blocks.get(i).getLength()));

                    tempF_IP = cplex_IP.diff(tempF_IP, Ie_it_IP[n][i][T - 1]);
                    tempF_IP = cplex_IP.sum(tempF_IP, B_it_IP[n][i][T - 1]);
                    tempF_IP = cplex_IP.sum(tempF_IP, scenarioTree.tree.get(T - 1).getCurLayer().get(d).get(blocks.get(i).getLength()));

                    tempC += scenarioTree.tree.get(T - 1).getCurLayer().get(d).get(blocks.get(i).getLength());
                    for (int k = 0; k < rawMaterials.size(); k++) {
                        for (int j = 0; j < allPatterns.get(k).size(); j++) {
                            tempF = cplex.diff(tempF, cplex.prod(a_ijk[k][j][i], Q_ijt[n][k][j][T - 1 - Parameters.leadTime]));
                        }
                    }
                    tempF = cplex.prod(tempF, duals[d][i]);
                    tempF_IP = cplex.prod(tempF_IP, duals[d][i]);
                    tempC *= duals[d][i];
                    Fl = cplex.sum(Fl, tempF);
                    Fl_IP = cplex_IP.sum(Fl_IP, tempF_IP);
                    tempC1 += tempC;
                }
                rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
                rhs_IP = cplex_IP.sum(rhs_IP, cplex.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
                cons += tempC1 / Parameters.nodeNumPerLayer;
            }
            RHS_Const[n][T-1][iter-1] = cons;
            cutRanges[n][T-1][iter-1] = cplex.addGe(cplex.diff(exp, rhs), 0, "benders_cut"+(n+1)+","+T+","+iter);
            cutRanges_IP[n][T-1][iter-1] = cplex_IP.addGe(cplex_IP.diff(exp_IP, rhs_IP), 0, "benders_cut"+(n+1)+","+T+","+iter+",IP");
            cuts[n][T-1][iter-1] = cutRanges[n][T-1][iter-1].toString();
            if(cplex.solve()) System.out.println("T周期添加benders后"+cplex.getObjValue());
            System.out.println(cplex.getValue(F[n][T-1]));
        }

        //开始为T-1周期及之前的周期生成cut
        for(int t=T;t>2;t--) {
            double[][] stockDuals = new double[Parameters.nodeNumPerLayer][];
            double[][] prodDuals = new double[Parameters.nodeNumPerLayer][];
            double[][] demandDuals = new double[Parameters.nodeNumPerLayer][];
            double[][] cutDuals = new double[Parameters.nodeNumPerLayer][Parameters.maxIter];
            solver = new ModelSolver(t-1, models);
            solver.solve();
            for (int i = 0; i < Parameters.nodeNumPerLayer; i++) {
                //if (!models[i][t - 1].solve()) throw new Exception("反向传播模型求解失败");
//                System.out.println(models[i][t-1].getObjValue());
//                System.out.println(models[i][t-1].getValue(F[i][t-1]));
//                for(int j=0;j< blocks.size();j++){
//                    System.out.println(models[i][t-1].getValue(Ie_it[i][j][t-1]));
//                    System.out.println(models[i][t-1].getValue(B_it[i][j][t-1]));
//                }
                stockDuals[i] = models[i][t - 1].getDuals(blockStockRanges[i][t - 1]);
                prodDuals[i] = models[i][t - 1].getDuals(materialProdRanges[i][t - 1]);
                demandDuals[i] = models[i][t - 1].getDuals(demandUBRanges[i][t - 1]);
                for (int k = 0; k < iter; k++) {
                    cutDuals[i][k] = models[i][t - 1].getDual(cutRanges[i][t - 1][k]);
                }
            }
            for (int n = 0; n < Parameters.nodeNumPerLayer; n++) {
                IloCplex cplex = models[n][t - 2];
                IloCplex cplex_IP = models_IP[n][t-2];
                IloNumExpr exp = cplex.numExpr();
                exp = cplex.sum(exp, F[n][t - 2]);
                IloNumExpr rhs = cplex.numExpr();
                IloNumExpr exp_IP = cplex_IP.numExpr();
                exp_IP = cplex_IP.sum(exp_IP, F_IP[n][t - 2]);
                IloNumExpr rhs_IP = cplex_IP.numExpr();
                double cons = 0;
                for (int d = 0; d < Parameters.nodeNumPerLayer; d++) {
                    IloNumExpr Fl = cplex.numExpr();
                    IloNumExpr Fl_IP = cplex_IP.numExpr();
                    double tempC1 = 0;
                    for (int i = 0; i < blocks.size(); i++) {
                        IloNumExpr tempF = cplex.numExpr();
                        IloNumExpr tempF_IP = cplex_IP.numExpr();
                        double tempC = 0;
                        tempF = cplex.diff(tempF, Ie_it[n][i][t - 2]);
                        tempF = cplex.sum(tempF, B_it[n][i][t-2]);
                        tempF_IP = cplex_IP.diff(tempF_IP, Ie_it_IP[n][i][t - 2]);
                        tempF_IP = cplex_IP.sum(tempF_IP, B_it_IP[n][i][t-2]);
                        for (int k = 0; k < rawMaterials.size(); k++) {
                            for (int j = 0; j < allPatterns.get(k).size(); j++) {
                                tempF = cplex.diff(tempF, cplex.prod(a_ijk[k][j][i], Q_ijt[n][k][j][t-2-Parameters.leadTime]));
                                tempF_IP = cplex_IP.diff(tempF_IP, cplex_IP.prod(a_ijk[k][j][i], Q_ijt_IP[n][k][j][t-2-Parameters.leadTime]));
                            }
                        }
                        tempC += scenarioTree.tree.get(t - 2).getCurLayer().get(d).get(blocks.get(i).getLength());
                        tempF = cplex.sum(tempF, scenarioTree.tree.get(t - 2).getCurLayer().get(d).get(blocks.get(i).getLength()));
                        tempF = cplex.prod(tempF, stockDuals[d][i]);
                        tempF_IP = cplex_IP.sum(tempF_IP, scenarioTree.tree.get(t - 2).getCurLayer().get(d).get(blocks.get(i).getLength()));
                        tempF_IP = cplex_IP.prod(tempF_IP, stockDuals[d][i]);
                        tempC *= stockDuals[d][i];
                        Fl = cplex.sum(Fl, tempF);
                        Fl_IP = cplex_IP.sum(Fl_IP, tempF_IP);
                        tempC1 += tempC;
                    }
                    rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
                    rhs_IP = cplex_IP.sum(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
                    cons += tempC1 / Parameters.nodeNumPerLayer;

                    Fl = cplex.numExpr();
                    Fl_IP = cplex_IP.numExpr();
                    for (int i = 0; i < rawMaterials.size(); i++) {
                        Fl = cplex.sum(Fl, cplex.prod(prodDuals[d][i], Ic_it[n][i][t - 2]));
                        Fl_IP = cplex_IP.sum(Fl_IP, cplex_IP.prod(prodDuals[d][i], Ic_it_IP[n][i][t - 2]));
                    }
                    rhs = cplex.diff(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
                    rhs_IP = cplex_IP.diff(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));

                    Fl = cplex.numExpr();
                    Fl_IP = cplex_IP.numExpr();
                    tempC1 = 0;
                    for(int i=0;i<blocks.size();i++){
                        Fl = cplex.sum(Fl, demandDuals[d][i]* worstDemand.get(i));
                        Fl_IP = cplex_IP.sum(Fl_IP, demandDuals[d][i]* worstDemand.get(i));
                        tempC1 += demandDuals[d][i]* worstDemand.get(i);
                    }
                    rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
                    rhs_IP = cplex_IP.sum(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
                    cons += tempC1 / Parameters.nodeNumPerLayer;

                    Fl = cplex.numExpr();
                    Fl_IP = cplex_IP.numExpr();
                    tempC1 = 0;
                    for(int i=0;i<iter;i++){
                        Fl = cplex.sum(Fl, RHS_Const[n][t-1][i]*cutDuals[d][i]);
                        Fl_IP = cplex_IP.sum(Fl_IP, RHS_Const[n][t-1][i]*cutDuals[d][i]);
                        tempC1 += RHS_Const[n][t-1][i]*cutDuals[d][i];
                    }
                    rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
                    rhs_IP = cplex_IP.sum(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
                    cons += tempC1 / Parameters.nodeNumPerLayer;
                }
                RHS_Const[n][t - 2][iter - 1] = cons;
                cutRanges[n][t - 2][iter - 1] = cplex.addGe(cplex.diff(exp, rhs), 0, "benders_cut"+(n+1)+","+(t-1)+","+iter);
                cutRanges_IP[n][t - 2][iter - 1] = cplex_IP.addGe(cplex_IP.diff(exp_IP, rhs_IP), 0, "benders_cut"+(n+1)+","+(t-1)+","+iter+",IP");
                cuts[n][t-2][iter-1] = cutRanges[n][t-2][iter-1].toString();
            }
        }
        //接下来向t=1传播
        double[][] stockDuals = new double[Parameters.nodeNumPerLayer][];
        double[][] prodDuals = new double[Parameters.nodeNumPerLayer][];
        double[][] demandDuals = new double[Parameters.nodeNumPerLayer][];
        double[][] cutDuals = new double[Parameters.nodeNumPerLayer][Parameters.maxIter];
        solver = new ModelSolver(1, models);
        solver.solve();
        for (int i = 0; i < Parameters.nodeNumPerLayer; i++) {
            //if (!models[i][1].solve()) throw new Exception("反向传播模型求解失败");
            stockDuals[i] = models[i][1].getDuals(blockStockRanges[i][1]);
            prodDuals[i] = models[i][1].getDuals(materialProdRanges[i][1]);
            demandDuals[i] = models[i][1].getDuals(demandUBRanges[i][1]);
            for (int k = 0; k < iter; k++) {
                cutDuals[i][k] = models[i][1].getDual(cutRanges[i][1][k]);
            }
        }
        IloCplex cplex = models[0][0];
        IloNumExpr exp = cplex.numExpr();
        exp = cplex.sum(exp, F[0][0]);
        IloNumExpr rhs = cplex.numExpr();
        IloCplex cplex_IP = models_IP[0][0];
        IloNumExpr exp_IP = cplex_IP.numExpr();
        exp_IP = cplex_IP.sum(exp_IP, F_IP[0][0]);
        IloNumExpr rhs_IP = cplex_IP.numExpr();
        double cons = 0;
        for (int d = 0; d < Parameters.nodeNumPerLayer; d++) {
            IloNumExpr Fl = cplex.numExpr();
            IloNumExpr Fl_IP = cplex_IP.numExpr();
            double tempC1 = 0;
            for (int i = 0; i < blocks.size(); i++) {
                IloNumExpr tempF = cplex.numExpr();
                IloNumExpr tempF_IP = cplex_IP.numExpr();
                double tempC = 0;
                tempF = cplex.diff(tempF, Ie_it[0][i][0]);
                tempF = cplex.sum(tempF, B_it[0][i][0]);
                tempF_IP = cplex_IP.diff(tempF_IP, Ie_it_IP[0][i][0]);
                tempF_IP = cplex_IP.sum(tempF_IP, B_it_IP[0][i][0]);
                for (int k = 0; k < rawMaterials.size(); k++) {
                    for (int j = 0; j < allPatterns.get(k).size(); j++) {
                        tempF = cplex.diff(tempF, cplex.prod(a_ijk[k][j][i], Q_ijt[0][k][j][0]));
                        tempF_IP = cplex_IP.diff(tempF_IP, cplex_IP.prod(a_ijk[k][j][i], Q_ijt_IP[0][k][j][0]));
                    }
                }
                tempC += scenarioTree.tree.get(0).getCurLayer().get(d).get(blocks.get(i).getLength());
                tempF = cplex.sum(tempF, scenarioTree.tree.get(0).getCurLayer().get(d).get(blocks.get(i).getLength()));
                tempF = cplex.prod(tempF, stockDuals[d][i]);
                tempF_IP = cplex_IP.sum(tempF_IP, scenarioTree.tree.get(0).getCurLayer().get(d).get(blocks.get(i).getLength()));
                tempF_IP = cplex_IP.prod(tempF_IP, stockDuals[d][i]);
                tempC *= stockDuals[d][i];
                Fl = cplex.sum(Fl, tempF);
                Fl_IP = cplex_IP.sum(Fl_IP, tempF_IP);
                tempC1 += tempC;
            }
            rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
            rhs_IP = cplex_IP.sum(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
            cons += tempC1 / Parameters.nodeNumPerLayer;

            Fl = cplex.numExpr();
            Fl_IP = cplex_IP.numExpr();
            for (int i = 0; i < rawMaterials.size(); i++) {
                Fl = cplex.sum(Fl, cplex.prod(prodDuals[d][i], Ic_it[0][i][0]));
                Fl_IP = cplex_IP.sum(Fl_IP, cplex_IP.prod(prodDuals[d][i], Ic_it_IP[0][i][0]));
            }
            rhs = cplex.diff(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
            rhs_IP = cplex_IP.diff(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));

            Fl = cplex.numExpr();
            Fl_IP = cplex_IP.numExpr();
            tempC1 = 0;
            for(int i=0;i<blocks.size();i++){
                Fl = cplex.sum(Fl, demandDuals[d][i]* worstDemand.get(i));
                Fl_IP = cplex_IP.sum(Fl_IP, demandDuals[d][i]* worstDemand.get(i));
                tempC1 += demandDuals[d][i]* worstDemand.get(i);
            }
            rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
            rhs_IP = cplex_IP.sum(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
            cons += tempC1 / Parameters.nodeNumPerLayer;

            Fl = cplex.numExpr();
            Fl_IP = cplex_IP.numExpr();
            tempC1 = 0;
            for(int i=0;i<iter;i++){
                Fl = cplex.sum(Fl, RHS_Const[0][1][i]*cutDuals[d][i]);
                Fl_IP = cplex_IP.sum(Fl_IP, RHS_Const[0][1][i]*cutDuals[d][i]);
                tempC1 += RHS_Const[0][1][i]*cutDuals[d][i];
            }
            rhs = cplex.sum(rhs, cplex.prod(Fl, (double) 1 / Parameters.nodeNumPerLayer));
            rhs_IP = cplex_IP.sum(rhs_IP, cplex_IP.prod(Fl_IP, (double) 1 / Parameters.nodeNumPerLayer));
            cons += tempC1 / Parameters.nodeNumPerLayer;
        }
        RHS_Const[0][0][iter - 1] = cons;
        cutRanges[0][0][iter - 1] = cplex.addGe(cplex.diff(exp, rhs), 0, "benders_cut"+"1,1"+","+iter);
        cutRanges_IP[0][0][iter - 1] = cplex_IP.addGe(cplex_IP.diff(exp_IP, rhs_IP), 0, "benders_cut"+"1,1"+","+iter+",IP");
        cuts[0][0][iter-1] = cutRanges[0][0][iter-1].toString();

    }
    public  CplexModel(ArrayList<Material> rawMaterials, ArrayList<Block> blocks, int T, ScenarioTree scenarioTree) throws IloException {
        this.T = T;
        this.rawMaterials = rawMaterials;
        this.blocks = blocks;
        this.scenarioTree = scenarioTree;
        this.models = new IloCplex[Parameters.nodeNumPerLayer][T+1];
        this.models_IP = new IloCplex[Parameters.nodeNumPerLayer][T+1];
        this.Y_i = new IloNumVar[rawMaterials.size()];
        this.Ic_it = new IloNumVar[Parameters.nodeNumPerLayer][][];   //[0,T-1]
        this.Ie_it = new IloNumVar[Parameters.nodeNumPerLayer][][];   //[0,T]
        this.Q_ijt = new IloNumVar[Parameters.nodeNumPerLayer][][][];   //[0,T-1]
        this.B_it = new IloNumVar[Parameters.nodeNumPerLayer][][];   //[0,T]
        this.F = new IloNumVar[Parameters.nodeNumPerLayer][T];  //[0,T-1]
        this.Y_i_IP = new IloIntVar[rawMaterials.size()];
        this.Ic_it_IP = new IloIntVar[Parameters.nodeNumPerLayer][][];   //[0,T-1]
        this.Ie_it_IP = new IloIntVar[Parameters.nodeNumPerLayer][][];   //[0,T]
        this.Q_ijt_IP = new IloIntVar[Parameters.nodeNumPerLayer][][][];   //[0,T-1]
        this.B_it_IP = new IloIntVar[Parameters.nodeNumPerLayer][][];   //[0,T]
        this.F_IP = new IloIntVar[Parameters.nodeNumPerLayer][T];  //[0,T-1]
        this.blockStockRanges = new IloRange[Parameters.nodeNumPerLayer][][];  //[0,T]
        this.materialProdRanges = new IloRange[Parameters.nodeNumPerLayer][][];  //[0,T-1]
        this.demandUBRanges = new IloRange[Parameters.nodeNumPerLayer][][];   //[0,T-1]
        this.cutRanges = new IloRange[Parameters.nodeNumPerLayer][][];   //[0,T-1]
        this.blockStockRanges_IP = new IloRange[Parameters.nodeNumPerLayer][][];  //[0,T]
        this.materialProdRanges_IP = new IloRange[Parameters.nodeNumPerLayer][][];  //[0,T-1]
        this.demandUBRanges_IP = new IloRange[Parameters.nodeNumPerLayer][][];   //[0,T-1]
        this.cutRanges_IP = new IloRange[Parameters.nodeNumPerLayer][][];   //[0,T-1]
        this.RHS_Const = new double[Parameters.nodeNumPerLayer][][];
        this.cuts = new String[Parameters.nodeNumPerLayer][][];
        //原材料通过长度区分规格
        allPatterns = new ArrayList<>();
        for (Material rawMaterial : rawMaterials) {
            allPatterns.add(patternsGenerate(rawMaterial, blocks));
        }
        for(int i=0;i<Parameters.nodeNumPerLayer;i++){
            this.RHS_Const[i] = new double[T][];
            this.cuts[i] = new String[T][];
            this.Q_ijt[i] = new IloNumVar[rawMaterials.size()][][];
            this.Ic_it[i] = new IloNumVar[rawMaterials.size()][];
            this.Ie_it[i] = new IloNumVar[blocks.size()][];
            this.B_it[i] = new IloNumVar[blocks.size()][];
            this.blockStockRanges[i] = new IloRange[T+1][];
            this.materialProdRanges[i] = new IloRange[T][];
            this.demandUBRanges[i] = new IloRange[T][];
            this.cutRanges[i] = new IloRange[T][];
            this.Q_ijt_IP[i] = new IloIntVar[rawMaterials.size()][][];
            this.Ic_it_IP[i] = new IloIntVar[rawMaterials.size()][];
            this.Ie_it_IP[i] = new IloIntVar[blocks.size()][];
            this.B_it_IP[i] = new IloIntVar[blocks.size()][];
            this.blockStockRanges_IP[i] = new IloRange[T+1][];
            this.materialProdRanges_IP[i] = new IloRange[T][];
            this.demandUBRanges_IP[i] = new IloRange[T][];
            this.cutRanges_IP[i] = new IloRange[T][];
            for(int t=0;t<T;t++){
                this.RHS_Const[i][t] = new double[Parameters.maxIter];
                this.cutRanges[i][t] = new IloRange[Parameters.maxIter];
                this.materialProdRanges[i][t] = new IloRange[rawMaterials.size()];
                this.demandUBRanges[i][t] = new IloRange[blocks.size()];
                this.cuts[i][t] = new String[Parameters.maxIter];
                this.cutRanges_IP[i][t] = new IloRange[Parameters.maxIter];
                this.materialProdRanges_IP[i][t] = new IloRange[rawMaterials.size()];
                this.demandUBRanges_IP[i][t] = new IloRange[blocks.size()];
            }
            for(int t=0;t<T+1;t++){
                this.blockStockRanges[i][t] = new IloRange[blocks.size()];
                this.blockStockRanges_IP[i][t] = new IloRange[blocks.size()];
            }
            for(int j=0;j<rawMaterials.size();j++){
                this.Ic_it[i][j] = new IloNumVar[T];
                this.Ic_it_IP[i][j] = new IloIntVar[T];
            }
            for(int j=0;j<blocks.size();j++){
                this.Ie_it[i][j] = new IloNumVar[T+1];
                this.B_it[i][j] = new IloNumVar[T+1];
                this.Ie_it_IP[i][j] = new IloIntVar[T+1];
                this.B_it_IP[i][j] = new IloIntVar[T+1];
            }
            for(int j=0;j< rawMaterials.size();j++){
                this.Q_ijt[i][j] = new IloNumVar[allPatterns.get(j).size()][];
                this.Q_ijt_IP[i][j] = new IloIntVar[allPatterns.get(j).size()][];
                for(int p=0;p<allPatterns.get(j).size();p++){
                    this.Q_ijt[i][j][p] = new IloNumVar[T];
                    this.Q_ijt_IP[i][j][p] = new IloIntVar[T];
                }
            }
        }
        //需求矩阵D_it的生成
        ArrayList<HashMap<Integer, Integer>> demandAverage = new ArrayList<>();
        for(int i=0;i<T;i++){
            HashMap<Integer, Integer> curDemandAverage = new HashMap<>();
            for (Block block : blocks) {
                curDemandAverage.put(block.getLength(), block.getDemandAverage().get(i));
            }
            demandAverage.add(curDemandAverage);
        }
        scenarioTree.buildTree(Parameters.nodeNumPerLayer, T, demandAverage);
        ArrayList<Integer> demands = scenarioTree.samplePath();
        D_it = new int[blocks.size()][T];
        for(int i=0;i<T;i++){
            for(int j=0;j<blocks.size();j++){
                D_it[j][i] = scenarioTree.tree.get(i).getCurLayer().get(demands.get(i)).get(blocks.get(j).getLength());
            }
        }
        //切割方案矩阵a_ijk的生成
        a_ijk = new int[rawMaterials.size()][][];
        for(int i=0;i<allPatterns.size();i++){
            a_ijk[i] = new int[allPatterns.get(i).size()][];
            for(int j=0;j<a_ijk[i].length;j++){
                a_ijk[i][j] = new int[blocks.size()];
                for(int k=0;k<a_ijk[i][j].length;k++){
                    a_ijk[i][j][k] = allPatterns.get(i).get(j).scheme.getOrDefault(blocks.get(k).getLength(), 0);
                }
            }
        }
    }
    public static ArrayList<Pattern> patternsGenerate(Material curMaterial, ArrayList<Block> curBlocks){
        //为当前规格原料生成所有切割方案
        ArrayList<Integer> blockLengths = new ArrayList<>();
        for(Block b : curBlocks) blockLengths.add(b.getLength());
        ArrayList<ArrayList<Integer>> allPatterns = new ArrayList<>();
        generate(curMaterial.getLength(), blockLengths, 0, new ArrayList<>(), allPatterns);
        ArrayList<Pattern> patterns = new ArrayList<>();
        for (ArrayList<Integer> allPattern : allPatterns) {
            Pattern pattern = new Pattern();
            HashMap<Integer, Integer> pattern_map = new HashMap<>();
            for (Integer integer : allPattern) {
                if (pattern_map.containsKey(integer))
                    pattern_map.put(integer, pattern_map.get(integer) + 1);
                else pattern_map.put(integer, 1);
            }
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

    public static void main(String[] args) {

    }
}



