import ilog.concert.IloException;
import ilog.concert.IloNumExpr;
import ilog.concert.IloNumVar;
import ilog.concert.IloRange;
import ilog.cplex.IloCplex;

import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.PrintStream;

public class test {
    public static void main(String[] args) throws Exception{
        FileOutputStream log = new FileOutputStream("logTest.log");
        PrintStream ps = new PrintStream(log);
        System.setOut(ps);

        for(int t=5-1;t>=2;t--){
            System.out.println(t);
        }
        try {
            IloCplex cplex = new IloCplex();
            IloNumVar x1 = cplex.numVar(0, Integer.MAX_VALUE, "x1");
            IloNumVar x2 = cplex.numVar(0, Integer.MAX_VALUE, "x2");
            IloNumExpr obj = cplex.numExpr();
            obj = cplex.sum(obj, cplex.prod(3, x1));
            obj = cplex.sum(obj, cplex.prod(2, x2));
            cplex.addMinimize(obj);
            IloRange[] range = new IloRange[2];
            IloNumExpr c1 = cplex.numExpr();
//            c1 = cplex.sum(c1, cplex.prod(1, x1));
//            c1 = cplex.sum(c1, cplex.prod(1, x2));
//            range[0] = cplex.addEq(c1, 4);
            c1 = cplex.diff(4, x1);
            c1 = cplex.diff(c1, x2);
            range[0] = cplex.addEq(c1, 0);
            IloNumExpr c2 = cplex.numExpr();
            c2 = cplex.sum(c2, cplex.prod(x1, 1));
            c2 = cplex.sum(c2, cplex.prod(x2, 2));
            c2 = cplex.diff(c2, cplex.prod(x2, 2));
            range[1] = cplex.addLe(c2, 6);
            if(cplex.solve()){
                cplex.exportModel("qwe.lp");
                System.out.println(cplex.getObjValue());
                System.out.println(cplex.getValue(x1));
                System.out.println(cplex.getValue(x2));
                System.out.println(cplex.getDuals(range)[0]);
                System.out.println(cplex.getDuals(range)[1]);
            }
        }catch (IloException e){
            e.printStackTrace();
        }
    }
}
