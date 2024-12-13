import java.io.FileOutputStream;
import java.io.PrintStream;
import java.time.LocalDateTime;
import java.time.format.DateTimeFormatter;
import java.util.ArrayList;

public class Main {
    public static void main(String[] args) throws Exception {
        long startTime = System.currentTimeMillis();
        String mPath = "src/data/materials.csv";
        String bPath = "src/data/blocks.csv";
        String mPath1 = "src/data/materials_demo.csv";
        String bPath1 = "src/data/blocks_demo.csv";
        DateTimeFormatter dtf = DateTimeFormatter.ofPattern("yyyy-MM-dd_HH-mm-ss");
        String timestamp = LocalDateTime.now().format(dtf);
        FileOutputStream log = new FileOutputStream("demo_" + timestamp + ".log");
        PrintStream ps = new PrintStream(log);
        //System.setOut(ps);
        System.out.println("运行日志"+timestamp);
        DataRead dataRead = new DataRead(mPath, bPath);
        ScenarioTree scenarioTree = new ScenarioTree(dataRead.blocks);
        //Model3 cplexModel = new Model3(dataRead.rawMaterials, dataRead.blocks, dataRead.T, scenarioTree);
        CplexModel cplexModel = new CplexModel(dataRead.rawMaterials, dataRead.blocks, dataRead.T, scenarioTree);
        ArrayList<Integer> path =  scenarioTree.samplePath();
        try {
            ArrayList<Double> o = new ArrayList<>();
            Solution solution = cplexModel.forwardPass_Setup(dataRead.T, path);
            if(cplexModel.models[0][0].solve()) {
                o.add(cplexModel.models[0][0].getObjValue());
            }
            cplexModel.backwardPass(path, solution,1);
            for(int i=0;i<Parameters.maxIter-1;i++) {
                System.out.println("-------------------开始第"+(i+2)+"次迭代--------------------");
                ArrayList<Integer> path2 = scenarioTree.samplePath();
                if (cplexModel.models[0][0].solve()) {
                    o.add(cplexModel.models[0][0].getObjValue());
                }
                solution = cplexModel.forwardPass(dataRead.T, path2);
                cplexModel.backwardPass(path2, solution, i+2);
            }
            System.out.println("求解完成，目标函数变化如下");
            for(Double i : o){
                System.out.print(i+" ");
            }
            System.out.println();
            System.out.println("求解用时"+(System.currentTimeMillis()-startTime)/1000+"s");
        }catch (Exception e){
            e.printStackTrace();
        }
    }
}
