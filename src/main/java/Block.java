import java.util.ArrayList;

public class Block {
    //零件
    private final String type;  //类型
    private final int length;  //长度
    private final double hold_cost;  //库存成本
    private final double back_cost;  //缺货成本
    private final ArrayList<Integer> demandAverage;  //每个周期的需求量均值

    public Block(String type, int length, double hold_cost, double back_cost, ArrayList<Integer> demandAverage) {
        this.type = type;
        this.length = length;
        this.hold_cost = hold_cost;
        this.back_cost = back_cost;
        this.demandAverage = demandAverage;
    }

    public String getType() {
        return type;
    }

    public ArrayList<Integer> getDemandAverage() {
        return demandAverage;
    }

    public int getLength() {
        return length;
    }

    public double getHold_cost() {
        return hold_cost;
    }

    public double getBack_cost() {
        return back_cost;
    }

    public Block blockCopy(Block block) {
        return new Block(block.type, block.length, block.hold_cost, block.back_cost, block.demandAverage);
    }
}
