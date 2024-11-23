import ilog.concert.IloException;

import java.util.ArrayList;
import java.util.HashMap;

public class Main {
    public static void main(String[] args) {
        String mPath = "C:\\Users\\Rain\\Desktop\\graduation24\\src\\data\\materials_demo.csv";
        String bPath = "C:\\Users\\Rain\\Desktop\\graduation24\\src\\data\\blocks_demo.csv";
        DataRead dataRead = new DataRead(mPath, bPath);
        ScenarioTree scenarioTree = new ScenarioTree(dataRead.blocks);
        //Cplex_model cplexModel = new Cplex_model(dataRead.rawMaterials, dataRead.blocks, dataRead.T, scenarioTree);
        Model3 cplexModel = new Model3(dataRead.rawMaterials, dataRead.blocks, dataRead.T, scenarioTree);
        ArrayList<Integer> path =  scenarioTree.samplePath();
        try {
            ArrayList<Double> o = new ArrayList<>();
            Solution solution = cplexModel.forwardPass_Setup(dataRead.T, path);
            if(cplexModel.models[0][0].solve()) o.add(cplexModel.models[0][0].getObjValue());
            cplexModel.backwardPass(path, solution,1);
            for(int i=0;i<18;i++) {
                ArrayList<Integer> path2 = scenarioTree.samplePath();
                if (cplexModel.models[0][0].solve()) o.add(cplexModel.models[0][0].getObjValue());
                System.out.println(cplexModel.models[0][0].getValue(cplexModel.F[0][0]));
                solution = cplexModel.forwardPass(dataRead.T, path2);
                cplexModel.backwardPass(path2, solution, i+2);
            }
            System.out.println();
        }catch (Exception e){
            e.printStackTrace();
        }
    }
}
