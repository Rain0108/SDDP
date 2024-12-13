import ilog.cplex.IloCplex;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.*;

public class ModelSolver {
    private final int t;
    private final IloCplex[][] models;

    public ModelSolver(int t, IloCplex[][] models) {
        this.t = t;
        this.models = models;
    }

    public void solve() throws InterruptedException, ExecutionException {
        ExecutorService executor = Executors.newFixedThreadPool(models.length);
        List<Future<Void>> futures = new ArrayList<>();

        for (int i = 0; i < models.length; i++) {
            final int layer = i;
            futures.add(executor.submit(() -> {
                new CplexSolver(layer, t, models).run();
                return null; // Callable 要求返回值
            }));
        }
        for (Future<Void> future : futures) {
            future.get(); // 阻塞直到任务完成
        }
        executor.shutdown();
        System.out.println("所有线程已完成求解");
    }
}

class CplexSolver implements Runnable {
    private final int layer;
    private final int t;
    private final IloCplex[][] models;

    public CplexSolver(int layer, int t, IloCplex[][] models) {
        this.layer = layer;
        this.t = t;
        this.models = models;
    }

    @Override
    public void run() {
        try {
            IloCplex model = models[layer][t];
            // 检查模型是否已初始化
            if (model == null) {
                throw new Exception("Model at layer " + layer + " and t " + t + " is not initialized.");
            }
            if (!model.solve()) {
                throw new Exception("Layer " + (layer + 1) + ", Period " + (t + 1) + ": No Solution exists.");
            } else {
                System.out.println(Thread.currentThread().getName() + " completed solving.");
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
