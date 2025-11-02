import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

public class ScenarioTree {
    //场景树的架构
    public static Random random = new Random(6);
    public ArrayList<Layer> tree;
    public ArrayList<Block> blocks;
    //生成场景路径（每个周期的需求）
    public ArrayList<HashMap<Integer, Integer>> generateNormal(ArrayList<HashMap<Integer, Integer>> demandMeanValue){
        // 正态采样，可通过 Parameters.demandStdFactor 调节波动强度
        ArrayList<HashMap<Integer, Integer>> res = new ArrayList<>();
        for(int i=0;i<demandMeanValue.size();i++){
            HashMap<Integer, Integer> map = new HashMap<>();
            for(Map.Entry<Integer, Integer> entry : demandMeanValue.get(i).entrySet()){
                double mu = entry.getValue();
                double sigma = Math.sqrt(Math.max(0.0, mu)) * Parameters.demandStdFactor;
                int val = (int) Math.round(mu + sigma * random.nextGaussian());
                map.put(entry.getKey(), Math.max(0, val));
            }
            res.add(map);
        }
        return res;
    }

    public ArrayList<HashMap<Integer, Integer>> generatePoisson(ArrayList<HashMap<Integer, Integer>> demandMeanValue){
        // 泊松采样 (λ=均值)
        ArrayList<HashMap<Integer, Integer>> res = new ArrayList<>();
        for(int i=0;i<demandMeanValue.size();i++){
            HashMap<Integer, Integer> map = new HashMap<>();
            for(Map.Entry<Integer, Integer> entry : demandMeanValue.get(i).entrySet()){
                int mu = Math.max(0, entry.getValue());
                int val = poisson(mu);
                map.put(entry.getKey(), val);
            }
            res.add(map);
        }
        return res;
    }

    private int poisson(double lambda){
        if(lambda <= 0) return 0;
        double L = Math.exp(-lambda);
        int k = 0;
        double p = 1.0;
        do {
            k++;
            p *= random.nextDouble();
        } while (p > L);
        return k - 1;
    }
    public void buildTree(int nodeNumPerLayer, int T, ArrayList<HashMap<Integer, Integer>> demandMeanValue){
        //建立场景树
        tree = new ArrayList<>();
        for(int t=0;t<T;t++) tree.add(new Layer(t, new ArrayList<>()));
        for(int i=0;i<nodeNumPerLayer;i++){
            ArrayList<HashMap<Integer, Integer>> curPath = Parameters.demandSampling.equals("poisson")
                    ? generatePoisson(demandMeanValue)
                    : generateNormal(demandMeanValue);
            for(int t=0;t<T;t++){
                tree.get(t).getCurLayer().add(curPath.get(t));
            }
        }
    }
    public ArrayList<Integer> samplePath(){
        //场景路径随机取样
        ArrayList<Integer> res = new ArrayList<>();
        for(int i=0;i<tree.size();i++){
            res.add(random.nextInt(tree.get(i).getCurLayer().size()));
        }
        return res;
    }
    public ArrayList<Integer> getWorstPath(){
        //返回需求均值最大的路径上每种成品的需求量总和
        //目前用均值来比较不同成品构成的需求向量
        ArrayList<Integer> res = new ArrayList<>();
        ArrayList<Integer> worst = new ArrayList<>();
        for(int t=0;t<tree.size();t++){
            int worstDemand = 0;
            int worstLoc = -1;
            for(int i=0;i<tree.get(t).getCurLayer().size();i++){
                int curDemand = 0;
                for(Map.Entry<Integer, Integer> entry : tree.get(t).getCurLayer().get(i).entrySet()){
                    curDemand += entry.getValue();
                }
                if(curDemand > worstDemand) {
                    worstLoc = i;
                    worstDemand = curDemand;
                }
            }
            worst.add(worstLoc);
        }
        for (Block block : blocks) {
            int curTotalDemand = 0;
            for (int t = 0; t < tree.size(); t++) {
                curTotalDemand += tree.get(t).getCurLayer().get(worst.get(t)).get(block.getLength());
            }
            res.add(curTotalDemand);
        }
        return res;
    }
    public ScenarioTree(ArrayList<Block> blocks){
        this.blocks = blocks;
    }
    public static void main(String[] args) {
        ArrayList<HashMap<Integer, Integer>> demands = new ArrayList<>();
        Random random1 = new Random();
        for(int i=0;i<5;i++){
            HashMap<Integer, Integer> map = new HashMap<>();
//            map.put("SA-213TP310HCbN", (random1.nextInt(100)+100));
//            map.put("SA-213S30432", (random1.nextInt(120)+100));
//            map.put("SA-213T91", (random1.nextInt(100)+50));
//            map.put("SA-213T92", (random1.nextInt(80)+50));
            demands.add(map);
        }
        //ScenarioTree sPathGenerator = new ScenarioTree();
        //ArrayList<HashMap<Integer, Integer>> res = sPathGenerator.generate(demands);
        //System.out.println();
    }
}
class Demand{
    Block block;
    int blockDemand;
    public Demand(Block block, int blockDemand){
        this.block = block.blockCopy(block);
        this.blockDemand = blockDemand;
    }

}
class Layer{
    //场景树按周期分层，每个节点等概率出现
    private final int t;
    private final ArrayList<HashMap<Integer, Integer>> curLayer;
    public Layer(int t, ArrayList<HashMap<Integer, Integer>> curLayer){
        this.t = t;
        this.curLayer = curLayer;
    }
    public int getT() {
        return t;
    }
    public ArrayList<HashMap<Integer, Integer>> getCurLayer(){
        return curLayer;
    }
}
