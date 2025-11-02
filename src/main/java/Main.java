import java.io.FileOutputStream;
import java.io.PrintStream;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;
import java.util.HashMap;

public class Main {
    public static void main(String[] args) throws Exception {
        long startTime = System.currentTimeMillis();
        String mPath1 = "src/data/materials_demo.csv";
        String bPath1 = "src/data/blocks_demo.csv";
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
        String timestamp = LocalDateTime.now().format(dtf);
        FileOutputStream log = new FileOutputStream("demo_" + timestamp + ".log");
        PrintStream ps = new PrintStream(log);
        //System.setOut(ps);
        System.out.println("运行日志"+timestamp);
        DataRead dataRead = new DataRead(mPath1, bPath1);
        ps.close();
        log.close();

        ScenarioTree scenarioTree = new ScenarioTree(dataRead.blocks);
        CplexModel cplexModel = new CplexModel(dataRead.rawMaterials, dataRead.blocks, dataRead.T, scenarioTree);
        ArrayList<Integer> path =  scenarioTree.samplePath();
        try {
            ArrayList<Double> o = new ArrayList<>();
            Solution solution = cplexModel.forwardPass_Setup(dataRead.T, path);
            if(cplexModel.models[0][0].solve()) {
                o.add(cplexModel.models[0][0].getObjValue());
            }
            cplexModel.backwardPass(path, solution,1);
            
            // 终止条件变量
            int consecutiveSmallChanges = 0;  // 连续小变动次数
            double tolerance = 1.0;  // 目标值变动容忍度
            long timeLimit = 1200 * 1000;  // 600秒时间限制（毫秒）
            boolean shouldStop = false;
            
            for(int i=0;i<Parameters.maxIter-1 && !shouldStop;i++) {
                // 检查时间限制
                long currentTime = System.currentTimeMillis();
                if(currentTime - startTime > timeLimit) {
                    System.out.println("达到600秒时间限制，停止迭代");
                    shouldStop = true;
                    break;
                }
                
                System.out.println("-------------------开始第"+(i+2)+"次迭代--------------------");
                ArrayList<Integer> path2 = scenarioTree.samplePath();
                if (cplexModel.models[0][0].solve()) {
                    double currentObjValue = cplexModel.models[0][0].getObjValue();
                    o.add(currentObjValue);
                    
                    // 检查目标值变动
                    if(o.size() >= 2) {
                        double prevObjValue = o.get(o.size() - 2);
                        double change = Math.abs(currentObjValue - prevObjValue);
                        
                        System.out.println("目标值变动: " + change + " (当前值: " + currentObjValue + ", 前值: " + prevObjValue + ")");
                        
                        if(change <= tolerance) {
                            consecutiveSmallChanges++;
                            System.out.println("连续小变动次数: " + consecutiveSmallChanges);
                        } else {
                            consecutiveSmallChanges = 0;
                        }
                        
                        // 检查连续5次小变动终止条件
                        if(consecutiveSmallChanges >= 5) {
                            System.out.println("连续5次迭代目标值变动不超过1，停止迭代");
                            shouldStop = true;
                            break;
                        }
                    }
                }
                solution = cplexModel.forwardPass(dataRead.T, path2);
                cplexModel.backwardPass(path2, solution, i+2);
//                if(i % 5 == 0) {
//                    System.out.println("----------------------开始模拟决策-----------------------");
//                    System.out.println(Functions.simulator(dataRead, cplexModel, solution));
//                }
            }
            System.out.println("求解完成，目标函数变化如下");
            for(Double i : o){
                System.out.print(i+" ");
            }
            System.out.println();
            System.out.println("求解用时"+(System.currentTimeMillis()-startTime)/1000+"s");
            double sddp_cost = Functions.simulator(dataRead, cplexModel, solution);

            ExpectedDemandModel model = new ExpectedDemandModel(dataRead.rawMaterials, dataRead.blocks, dataRead.T);
            model.buildIP();
            if (model.solveIP()) {
                System.out.println("期望需求IP模型求解完成，目标值=" + model.modelIP.getObjValue());
                // 如需导出LP文件查看模型：
                // model.exportIP("expected_ip.lp");

                // 提取确定性解决方案并进行多路径模拟评估
                DeterministicSolution detSolution = model.extractSolution();
                ScenarioTree scenarioTreeForDet = new ScenarioTree(dataRead.blocks);

                // 构建场景树用于模拟
                ArrayList<HashMap<Integer, Integer>> demandAverage = new ArrayList<>();
                for(int i=0;i<dataRead.T;i++){
                    HashMap<Integer, Integer> curDemandAverage = new HashMap<>();
                    for (Block block : dataRead.blocks) {
                        curDemandAverage.put(block.getLength(), block.getDemandAverage().get(i));
                    }
                    demandAverage.add(curDemandAverage);
                }
                scenarioTreeForDet.buildTree(Parameters.nodeNumPerLayer, dataRead.T, demandAverage);

                // 进行多路径模拟评估
                double deterministicCost = Functions.detModelSimulator(dataRead, cplexModel, detSolution);
                System.out.println("确定性模型多路径模拟评估成本=" + deterministicCost);
                System.out.printf("SDDP模拟评估成本=" + sddp_cost);
            }
        }catch (Exception e){
            e.printStackTrace();
        }
    }
}