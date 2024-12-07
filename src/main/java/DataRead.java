import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;

public class DataRead {
    public ArrayList<Material> rawMaterials;  //原材料集合
    public ArrayList<Block> blocks;  //零件集合
    public int T;
    public ArrayList<Material> readMaterials(String filePath){
        ArrayList<Material> res = new ArrayList<>();
        // 创建 reader
        try (BufferedReader br = Files.newBufferedReader(Paths.get(filePath))) {
            String DELIMITER = ",";
            String line;
            while ((line = br.readLine()) != null) {
                String[] columns = line.split(DELIMITER);
                if(columns[0].equals("type")) continue;
                res.add(new Material(columns[0], Integer.parseInt(columns[1]), Integer.parseInt(columns[2]), Integer.parseInt(columns[3])));
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        return res;
    }
    public ArrayList<Block> readBlocks(String filePath){
        ArrayList<Block> res = new ArrayList<>();
        // 创建 reader
        try (BufferedReader br = Files.newBufferedReader(Paths.get(filePath))) {
            String DELIMITER = ",";
            String line;
            while ((line = br.readLine()) != null) {
                String[] columns = line.split(DELIMITER);
                if(columns[0].equals("type")) {
                    T = Integer.parseInt(columns[columns.length-1]);
                    continue;
                }
                ArrayList<Integer> demandAverage = new ArrayList<>();
                for(int i=4;i<columns.length;i++){
                    demandAverage.add(Integer.parseInt(columns[i]));
                }
                try {
                    res.add(new Block(columns[0], Integer.parseInt(columns[1]), Double.parseDouble(columns[2]),
                            Double.parseDouble(columns[3]), demandAverage));
                    }catch (ArrayIndexOutOfBoundsException e){
                        System.out.println();
                }
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        return res;
    }
    public DataRead(String materialPath, String blockPath){
        System.out.println("开始数据读取");
        rawMaterials = readMaterials(materialPath);
        blocks = readBlocks(blockPath);
        System.out.println("完成数据读取");
    }
}
