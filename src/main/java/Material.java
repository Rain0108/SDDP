public class Material{
    //原材料
    private final String type;
    private final int length;
    private final double cost;
    private final double hold_cost;

    public Material(String type, int length, double cost, double hold_cost){
        this.type = type;
        this.length = length;
        this.cost = cost;
        this.hold_cost = hold_cost;
    }
    public String getType(){
        return type;
    }

    public double getCost() {
        return cost;
    }

    public int getLength() {
        return length;
    }

    public double getHold_cost() {
        return hold_cost;
    }
}
