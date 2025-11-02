import java.util.ArrayList;
import java.util.HashMap;

public class Functions {
    public static double simulator(DataRead dataRead, CplexModel model, Solution solution){
        int numPath = Parameters.simulatePaths;
        // 期初采购成本（所有路径共享，只计算一次）
        double procurementCost = 0;
        for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
            procurementCost += solution.Y_value_IP[i] * dataRead.rawMaterials.get(i).getCost();
        }
        
        double operationalCost = 0;
        
        // 为每条路径进行模拟
        for(int pathIndex = 0; pathIndex < numPath; pathIndex++) {
            // 生成新的场景路径
            ArrayList<Integer> path = model.scenarioTree.samplePath();
            // 初始化库存状态
            double[] materialInventory = new double[dataRead.rawMaterials.size()];
            double[] blockInventory = new double[dataRead.blocks.size()];
            
            // 设置第一期初的原材料库存（基于solution中的采购决策）
            for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                materialInventory[i] = solution.Y_value_IP[i];
            }
            // 切割生产阶段
            for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                for(int j = 0; j < model.allPatterns.get(i).size(); j++) {
                    // 获取切割方案使用量
                    double cutAmount = solution.Q_value_IP[0][i][j][0];

                    // 更新原材料库存
                    materialInventory[i] -= cutAmount;

                    // 添加切割成本
                    operationalCost += cutAmount * Parameters.cutCost;

                    // 根据切割方案生产零件
                    for(int k = 0; k < dataRead.blocks.size(); k++) {
                        double producedAmount = cutAmount * model.a_ijk[i][j][k];
                        blockInventory[k] += producedAmount;
                    }
                }
            }
            // 模拟每个周期的运营
            for(int t = 1; t < dataRead.T; t++) {
                // 获取上周期的需求
                HashMap<Integer, Integer> currentDemand = model.scenarioTree.tree.get(t-1).getCurLayer().get(path.get(t-1));
                
                // 切割生产阶段
                for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                    for(int j = 0; j < model.allPatterns.get(i).size(); j++) {
                        // 获取切割方案使用量
                        double cutAmount = solution.Q_value_IP[path.get(t-1)][i][j][t];
                        
                        // 更新原材料库存
                        materialInventory[i] -= cutAmount;
                        
                        // 添加切割成本
                        operationalCost += cutAmount * Parameters.cutCost;
                        
                        // 根据切割方案生产零件
                        for(int k = 0; k < dataRead.blocks.size(); k++) {
                            double producedAmount = cutAmount * model.a_ijk[i][j][k];
                            blockInventory[k] += producedAmount;
                        }
                    }
                }
                
                // 2. 需求满足阶段
                for(int k = 0; k < dataRead.blocks.size(); k++) {
                    int demand = currentDemand.getOrDefault(dataRead.blocks.get(k).getLength(), 0);
                    
                    // 优先用库存满足需求
                    double satisfiedFromInventory = Math.min(demand, blockInventory[k]);
                    blockInventory[k] -= satisfiedFromInventory;
                    
                    // 剩余需求直接购买成品（缺货成本）
                    double unmetDemand = demand - satisfiedFromInventory;
                    if(unmetDemand > 0) {
                        operationalCost += unmetDemand * dataRead.blocks.get(k).getBack_cost();
                    }
                }
                
                // 3. 库存持有成本
                for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                    operationalCost += materialInventory[i] * dataRead.rawMaterials.get(i).getHold_cost();
                }
                
                for(int k = 0; k < dataRead.blocks.size(); k++) {
                    operationalCost += blockInventory[k] * dataRead.blocks.get(k).getHold_cost();
                }
            }
            //期末
            HashMap<Integer, Integer> currentDemand = model.scenarioTree.tree.get(dataRead.T-1).getCurLayer().get(path.get(dataRead.T-1));
            for(int k = 0; k < dataRead.blocks.size(); k++) {
                int demand = currentDemand.getOrDefault(dataRead.blocks.get(k).getLength(), 0);

                // 优先用库存满足需求
                double satisfiedFromInventory = Math.min(demand, blockInventory[k]);
                blockInventory[k] -= satisfiedFromInventory;

                // 剩余需求直接购买成品（缺货成本）
                double unmetDemand = demand - satisfiedFromInventory;
                if(unmetDemand > 0) {
                    operationalCost += unmetDemand * dataRead.blocks.get(k).getBack_cost();
                }
            }
        }
        
        // 返回期初采购成本加上平均运营成本
        return procurementCost + (operationalCost / numPath);
    }

    public static double detModelSimulator(DataRead dataRead, CplexModel cplexModel, DeterministicSolution solution){
        int numPath = Parameters.simulatePaths;
        // 期初采购成本（所有路径共享，只计算一次）
        double procurementCost = 0;
        for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
            procurementCost += solution.Y_value_IP[i] * dataRead.rawMaterials.get(i).getCost();
        }

        double operationalCost = 0;

        // 为每条路径进行模拟
        for(int pathIndex = 0; pathIndex < numPath; pathIndex++) {
            // 生成新的场景路径
            ArrayList<Integer> path = cplexModel.scenarioTree.samplePath();
            // 初始化库存状态
            double[] materialInventory = new double[dataRead.rawMaterials.size()];
            double[] blockInventory = new double[dataRead.blocks.size()];

            // 设置第一期初的原材料库存（基于solution中的采购决策）
            for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                materialInventory[i] = solution.Y_value_IP[i];
            }
            // 切割生产阶段
            for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                for(int j = 0; j < cplexModel.allPatterns.get(i).size(); j++) {
                    // 获取切割方案使用量
                    double cutAmount = solution.Q_value_IP[i][j][0];

                    // 更新原材料库存
                    materialInventory[i] -= cutAmount;

                    // 添加切割成本
                    operationalCost += cutAmount * Parameters.cutCost;

                    // 根据切割方案生产零件
                    for(int k = 0; k < dataRead.blocks.size(); k++) {
                        double producedAmount = cutAmount * cplexModel.a_ijk[i][j][k];
                        blockInventory[k] += producedAmount;
                    }
                }
            }
            // 模拟每个周期的运营
            for(int t = 1; t < dataRead.T; t++) {
                // 获取上周期的需求
                HashMap<Integer, Integer> currentDemand = cplexModel.scenarioTree.tree.get(t-1).getCurLayer().get(path.get(t-1));

                // 切割生产阶段
                for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                    for(int j = 0; j < cplexModel.allPatterns.get(i).size(); j++) {
                        // 获取切割方案使用量
                        double cutAmount = solution.Q_value_IP[i][j][t];

                        // 更新原材料库存
                        materialInventory[i] -= cutAmount;

                        // 添加切割成本
                        operationalCost += cutAmount * Parameters.cutCost;

                        // 根据切割方案生产零件
                        for(int k = 0; k < dataRead.blocks.size(); k++) {
                            double producedAmount = cutAmount * cplexModel.a_ijk[i][j][k];
                            blockInventory[k] += producedAmount;
                        }
                    }
                }

                // 2. 需求满足阶段
                for(int k = 0; k < dataRead.blocks.size(); k++) {
                    int demand = currentDemand.getOrDefault(dataRead.blocks.get(k).getLength(), 0);

                    // 优先用库存满足需求
                    double satisfiedFromInventory = Math.min(demand, blockInventory[k]);
                    blockInventory[k] -= satisfiedFromInventory;

                    // 剩余需求直接购买成品（缺货成本）
                    double unmetDemand = demand - satisfiedFromInventory;
                    if(unmetDemand > 0) {
                        operationalCost += unmetDemand * dataRead.blocks.get(k).getBack_cost();
                    }
                }

                // 3. 库存持有成本
                for(int i = 0; i < dataRead.rawMaterials.size(); i++) {
                    operationalCost += materialInventory[i] * dataRead.rawMaterials.get(i).getHold_cost();
                }

                for(int k = 0; k < dataRead.blocks.size(); k++) {
                    operationalCost += blockInventory[k] * dataRead.blocks.get(k).getHold_cost();
                }
            }
            //期末
            HashMap<Integer, Integer> currentDemand = cplexModel.scenarioTree.tree.get(dataRead.T-1).getCurLayer().get(path.get(dataRead.T-1));
            for(int k = 0; k < dataRead.blocks.size(); k++) {
                int demand = currentDemand.getOrDefault(dataRead.blocks.get(k).getLength(), 0);

                // 优先用库存满足需求
                double satisfiedFromInventory = Math.min(demand, blockInventory[k]);
                blockInventory[k] -= satisfiedFromInventory;

                // 剩余需求直接购买成品（缺货成本）
                double unmetDemand = demand - satisfiedFromInventory;
                if(unmetDemand > 0) {
                    operationalCost += unmetDemand * dataRead.blocks.get(k).getBack_cost();
                }
            }
        }

        // 返回期初采购成本加上平均运营成本
        return procurementCost + (operationalCost / numPath);
    }
}