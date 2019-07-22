package net.jmecn.physics3d;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.plugins.FileLocator;
import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.BoxCollisionShape;
import com.jme3.export.binary.BinaryExporter;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;
import com.jme3.scene.Geometry;
import com.jme3.scene.Spatial;
import com.jme3.scene.shape.Box;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;
import com.jme3.texture.Texture;
import com.jme3.texture.Texture.WrapMode;
import com.jme3.math.*;
import com.jme3.scene.Mesh;
import java.io.File;
import java.io.IOException;
import sun.applet.Main;


public class HelloP extends SimpleApplication {

    public static void main(String[] args) {
        HelloP app = new HelloP();
        app.start();
    }
    private BulletAppState bulletAppState;
    

    
    
    
    

    @Override
    public void simpleInitApp() {
        
        
        bulletAppState = new BulletAppState();
        stateManager.attach(bulletAppState);
        assetManager.registerLocator("assets", FileLocator.class);
        BoxCollisionShape shape = new BoxCollisionShape(new Vector3f(3000f,0.1f,3000f));

        flyCam.setMoveSpeed(100);
        
        
        cam.setLocation(new Vector3f(0f,2f,-230f));   // camera start point
        
        
        
        cam.setRotation(new Quaternion(-0.046265673f, 0.9518722f, -0.1815604f, -0.2425582f));
        PhysicsSpace physicsSpace = bulletAppState.getPhysicsSpace(); 
        
        Mesh Grand = new Box (3000f,0.1f,3000f);           // grand box   
        Grand.scaleTextureCoordinates(new Vector2f(3000, 3000));
        Material Grand_brick = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Texture grasst = assetManager.loadTexture("Scenes/Task/two/grass.png");
        grasst.setWrap(WrapMode.Repeat);
        Grand_brick.setTexture("ColorMap", grasst);
        Geometry grand = new Geometry("floor", Grand);
        grand.setMaterial(Grand_brick);
        grand.setLocalTranslation(400, -0.11f, 398);
        Spatial gg = grand;
        rootNode.attachChild(gg);
        physicsSpace.add(gg);
        
        int num_road = 50;                      
        int num_sign = 50;
        int num_crosswalk = 50;
        int num_crossover = 5; 
        
        Spatial road[] = new Spatial[num_road];
        Material mat_brick1 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Texture roadt = assetManager.loadTexture("Scenes/Task/two/ML2.jpg");
        roadt.setWrap(WrapMode.Repeat);
        mat_brick1.setTexture("ColorMap", roadt);
        Box Road1 = new Box(4f,0.01f,200.0f);     // the first road box
        road[0] = new Geometry("floor", Road1); 
        road[0].setMaterial(mat_brick1);
        Road1.scaleTextureCoordinates(new Vector2f(23f, 1f));
        road[0].setLocalTranslation(0f,-0.01f,-50f);    // road 0
        rootNode.attachChild(road[0]);           
        
        Spatial roadLine[] = new Spatial[num_road];   
        Material mat_brick2 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Texture roadline = assetManager.loadTexture("Scenes/Task/two/s1_new.png");
        Material mat_brick6 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Texture roadline2 = assetManager.loadTexture("Scenes/Task/two/s2_new.png");
        Material mat_brick7 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Material mat_brick8 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        Texture roadline3 = assetManager.loadTexture("Scenes/Task/two/s3_new.png");
        roadline.setWrap(WrapMode.Repeat);
        Texture roadline4 = assetManager.loadTexture("Scenes/Task/two/s4_new.png");
        roadline.setWrap(WrapMode.Repeat);
        mat_brick2.setTexture("ColorMap", roadline);
        mat_brick6.setTexture("ColorMap", roadline2);
        mat_brick7.setTexture("ColorMap", roadline3);
        mat_brick8.setTexture("ColorMap", roadline4);
        Box Roadline = new Box(4f,0.01f,25.0f);                   // roadline box
        for(int i = 0; i<4; i++)   
        {
            roadLine[i] = new Geometry("floor", Roadline);
            roadLine[i].setMaterial(mat_brick2);
            roadLine[i+4] = new Geometry("floor", Roadline);
            roadLine[i+4].setMaterial(mat_brick6);
            roadLine[i+8] = new Geometry("floor", Roadline);
            roadLine[i+8].setMaterial(mat_brick7);
            roadLine[i+12] = new Geometry("floor", Roadline);
            roadLine[i+12].setMaterial(mat_brick8);
        } 
        Roadline.scaleTextureCoordinates(new Vector2f(1f, 1f));
        roadLine[0].setLocalTranslation(0f,-0.01f,175f);        // road0 line 0
        rootNode.attachChild(roadLine[0]);
//        roadLine[1].setLocalTranslation(0f,-0.01f,239f);        // road0 line 2
//        roadLine[1].rotate(0.0f,3.1415926f,0.0f);rootNode.attachChild(roadLine[1]);
//        roadLine[2].setLocalTranslation(32f,-0.01f,207f);        // road0 line 1
//        roadLine[2].rotate(0.0f,-1.57079f,0.0f);rootNode.attachChild(roadLine[2]);
//        roadLine[3].setLocalTranslation(-32f,-0.01f,207f);        // road0 line 3
//        roadLine[3].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(roadLine[3]);
        
        Box Crossover = new Box(4f,0.01f,4f);
        Spatial crossover[] = new Spatial[num_crossover];
        Material mat_brick4 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat_brick4.setTexture("ColorMap",
        assetManager.loadTexture("Scenes/Task/two/crossing.png"));
        for(int i = 0; i<num_crossover; i++)   
        {
            crossover[i] = new Geometry("floor", Crossover);
            crossover[i].setMaterial(mat_brick4);
        } 
        crossover[0].setLocalTranslation(0.0f,-0.01f,207f);           // crossover 0
        rootNode.attachChild(crossover[0]); 
        
        
        
        Spatial crossT[] = new Spatial[num_crossover];
        Material mat_brick5 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat_brick5.setTexture("ColorMap",
        assetManager.loadTexture("Scenes/Task/two/t-part.png"));
        for(int i = 0; i<num_crossover; i++)   
        {
            crossT[i] = new Geometry("floor", Crossover);
            crossT[i].setMaterial(mat_brick5);
        }   
        
  
        Box Crosswalk = new Box(4f,0.01f,1.5f);                       // cross walk box 
        Spatial crosswalk[] = new Spatial[num_crosswalk];
        Material mat_brick3 = new Material(
        assetManager, "Common/MatDefs/Misc/Unshaded.j3md");
        mat_brick3.setTexture("ColorMap",
        assetManager.loadTexture("Scenes/Task/two/crosswalk.png"));
        
        for(int i = 0; i<num_crosswalk; i++)   
        {
            crosswalk[i] = new Geometry("floor", Crosswalk);
            crosswalk[i].setMaterial(mat_brick3);
        }
        crosswalk[0].setLocalTranslation(0.0f,-0.01f,201.5f);rootNode.attachChild(crosswalk[0]);   // road0 crosswalk 0
        crosswalk[1].setLocalTranslation(0.0f,-0.01f,212.5f);rootNode.attachChild(crosswalk[1]);   // 0 2
        crosswalk[2].setLocalTranslation(-5.5f,-0.01f,207f);                                       // 0 3
        crosswalk[2].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(crosswalk[2]);
        crosswalk[3].setLocalTranslation(5.50f,-0.01f,207f);                                       // 0 4
        crosswalk[3].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(crosswalk[3]);
        
        
        
        
        
        
        
        Box Road2 = new Box(4f,0.01f,325.0f); // the second road condition
        Road2.scaleTextureCoordinates(new Vector2f(34f, 1f));
        for(int i = 1; i<num_road; i++)
        {
            road[i] = new Geometry("floor", Road2);  // the second road number 7
            road[i].setMaterial(mat_brick1);
        }

        road[1].setLocalTranslation(332f,-0.01f,207f);                    //  road 1
        road[1].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(road[1]); 
        road[4].setLocalTranslation(0.0f,-0.01f,539f);                    //  road 4
        rootNode.attachChild(road[4]);
        
        crosswalk[4].setLocalTranslation(719.5f,-0.01f,207f);
        crosswalk[4].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(crosswalk[4]);   // crossover 1 crosswalk
        crosswalk[5].setLocalTranslation(714f,-0.01f,212.5f);rootNode.attachChild(crosswalk[5]);   
        crosswalk[6].setLocalTranslation(708.5f,-0.01f,207f);                                       
        crosswalk[6].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(crosswalk[6]);   
        
        crossT[0].setLocalTranslation(714f,-0.01f,207f);           // crossover 1
        crossT[0].rotate(0.0f,-1.57079f,0.0f);rootNode.attachChild(crossT[0]);
        
//        roadLine[8].setLocalTranslation(746f,-0.01f,207f);        //  crossover 1 roadLine
//        roadLine[8].rotate(0.0f,-1.57079f,0.0f);rootNode.attachChild(roadLine[8]);
//        roadLine[12].setLocalTranslation(714f,-0.01f,239f);        // 
//        roadLine[12].rotate(0.0f,3.1415926f,0.0f);rootNode.attachChild(roadLine[12]);
        roadLine[4].setLocalTranslation(682f,-0.01f,207f);        // 
        roadLine[4].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(roadLine[4]);
        
        road[2].setLocalTranslation(714f,-0.01f,539f);                    //  road 2
        rootNode.attachChild(road[2]);
        
        
        roadLine[9].setLocalTranslation(0f,-0.01f,889f);        //  crossover 3 roadLine
        rootNode.attachChild(roadLine[9]);
//        roadLine[6].setLocalTranslation(0f,-0.01f,953f);        // 
//        roadLine[6].rotate(0.0f,3.1415926f,0.0f);rootNode.attachChild(roadLine[6]);
//        roadLine[13].setLocalTranslation(32f,-0.01f,921f);        // 
//        roadLine[13].rotate(0.0f,-1.57079f,0.0f);rootNode.attachChild(roadLine[13]);


        crosswalk[7].setLocalTranslation(0f,-0.01f,915.5f);rootNode.attachChild(crosswalk[7]);   // crossover 3 crosswalk
        crosswalk[8].setLocalTranslation(0f,-0.01f,926.5f);rootNode.attachChild(crosswalk[8]);   
        crosswalk[9].setLocalTranslation(5.5f,-0.01f,921f);                                       
        crosswalk[9].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(crosswalk[9]);  
        
        crossT[2].setLocalTranslation(0f,-0.01f,921f);           // crossover 3
        rootNode.attachChild(crossT[2]);
        
        
        road[3].setLocalTranslation(332f,-0.01f,921f);                    //  road 3
        road[3].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(road[3]);
        
        
        roadLine[5].setLocalTranslation(714f,-0.01f,889f);        //  crossover 2 roadLine
        rootNode.attachChild(roadLine[5]);
//        roadLine[10].setLocalTranslation(714f,-0.01f,953f);        // 
//        roadLine[10].rotate(0.0f,3.1415926f,0.0f);rootNode.attachChild(roadLine[10]);
        roadLine[14].setLocalTranslation(682f,-0.01f,921f);        // 
        roadLine[14].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(roadLine[14]);
        
        crosswalk[10].setLocalTranslation(714f,-0.01f,915.5f);rootNode.attachChild(crosswalk[10]);   // crossover 2 crosswalk
        crosswalk[11].setLocalTranslation(714f,-0.01f,926.5f);rootNode.attachChild(crosswalk[11]);   
        crosswalk[12].setLocalTranslation(708.5f,-0.01f,921f);                                       
        crosswalk[12].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(crosswalk[12]);   
        
        crossT[1].setLocalTranslation(714f,-0.01f,921f);           // crossover 2
        crossT[1].rotate(0.0f,3.1415926f,0.0f);rootNode.attachChild(crossT[1]);
        
        
        String path1 = "Scenes/Task/speedLimit50/speedLimit50.scene";
        assetManager.registerLocator("assets", FileLocator.class);
        Spatial speed_limit = (Spatial) assetManager.loadModel(path1);
        speed_limit.scale(2f, 2f, 2f);
        speed_limit.rotate(0f, 3.1415f, 0);
        speed_limit.setLocalTranslation(-5f, -1.1f, -200f);                               // model.setLocalTranslation(-3, -0f, -247);  Gate
        //model.setMaterial(Grand_brick);                                      //model.setLocalTranslation(700, -0f, -0);   BMW
        rootNode.attachChild(speed_limit);
        
        
        String path6 = "Scenes/Task/Parking_building/Parking.scene";
        assetManager.registerLocator("assets", FileLocator.class);
        Spatial parking = (Spatial) assetManager.loadModel(path6);
        parking.scale(1f, 1f, 1f);
        parking.rotate(0f, 0f, 0);
        parking.setLocalTranslation(714f, 8f, 1253f);                               // model.setLocalTranslation(-3, -0f, -247);  Gate
        //model.setMaterial(Grand_brick);                                      //model.setLocalTranslation(700, -0f, -0);   BMW
        rootNode.attachChild(parking);
        
        
        String path2 = "Scenes/Task/Roadsign4/RoadCLosed.scene";
        assetManager.registerLocator("assets", FileLocator.class);
        Spatial model_sign1 = (Spatial) assetManager.loadModel(path2);
        model_sign1.scale(0.015f, 0.015f, 0.015f);
        model_sign1.rotate(-1.57f, 1.57f, 0);
        model_sign1.setLocalTranslation(-5f,-2.1f,850f);                               // model.setLocalTranslation(-3, -0f, -247);  Gate
        //model.setMaterial(Grand_brick);                                      //model.setLocalTranslation(700, -0f, -0);   BMW
        rootNode.attachChild(model_sign1);
        
        String path3 = "Scenes/Task/Roadsign1/RoadCLosed.scene";
        assetManager.registerLocator("assets", FileLocator.class);
        Spatial model_sign2 = (Spatial) assetManager.loadModel(path3);
        model_sign2.scale(0.015f, 0.015f, 0.015f);
        model_sign2.rotate(-1.57f, 3.14f, 0);
        model_sign2.setLocalTranslation(650f,-2.01f,212f);                               // model.setLocalTranslation(-3, -0f, -247);  Gate
        //model.setMaterial(Grand_brick);                                      //model.setLocalTranslation(700, -0f, -0);   BMW
        rootNode.attachChild(model_sign2);
        
        String path4 = "Scenes/Task/Roadsign2/RoadCLosed.scene";
        assetManager.registerLocator("assets", FileLocator.class);
        Spatial model_sign3 = (Spatial) assetManager.loadModel(path4
        );
        model_sign3.scale(0.015f, 0.015f, 0.015f);
        model_sign3.rotate(-1.57f, 1.57f, 0);
        model_sign3.setLocalTranslation(709f,-2.01f,850f);                               // model.setLocalTranslation(-3, -0f, -247);  Gate
        //model.setMaterial(Grand_brick);                                      //model.setLocalTranslation(700, -0f, -0);   BMW
        rootNode.attachChild(model_sign3);
        
        String path5 = "Scenes/Task/Roadsign3/RoadCLosed.scene";
        assetManager.registerLocator("assets", FileLocator.class);
        Spatial model_sign4 = (Spatial) assetManager.loadModel(path3);
        model_sign4.scale(0.015f, 0.015f, 0.015f);
        model_sign4.rotate(-1.57f, 3.14f, 0);
        model_sign4.setLocalTranslation(650f,-2.01f,927f);                               // model.setLocalTranslation(-3, -0f, -247);  Gate
        //model.setMaterial(Grand_brick);                                      //model.setLocalTranslation(700, -0f, -0);   BMW
        rootNode.attachChild(model_sign4);
        
        
        road[5].setLocalTranslation(-332f,-0.01f,207f);                    //  road 5
        road[5].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(road[5]); 
        
        road[6].setLocalTranslation(1046f,-0.01f,207f);                    //  road 
        road[6].rotate(0.0f,1.57079f,0.0f);rootNode.attachChild(road[6]); 
        
        road[7].setLocalTranslation(0f,-0.01f,1253f);                    //  road 
        rootNode.attachChild(road[7]); 
        road[8].setLocalTranslation(714f,-0.01f,1253f);                    //  road 
        rootNode.attachChild(road[8]); 
        
        
  
//        
        DirectionalLight sun1 = new DirectionalLight();                         // light1
        sun1.setDirection(new Vector3f(0.1f, 0.7f, 1.0f).normalizeLocal());
        rootNode.addLight(sun1);
        DirectionalLight sun2 = new DirectionalLight();                         // light2
        sun2.setDirection(new Vector3f(1f, 0f, 1f).normalizeLocal());
        rootNode.addLight(sun2);
        DirectionalLight sun3 = new DirectionalLight();                         // light3
        sun3.setDirection(new Vector3f(-0.1f, -0.7f, -1.0f).normalizeLocal());
        rootNode.addLight(sun3);
        DirectionalLight sun4 = new DirectionalLight();                         // light4
        sun4.setDirection(new Vector3f(-1f, 0f, -1f).normalizeLocal());
        rootNode.addLight(sun4);        
//
//        
//        //Spatial sky =SkyFactory.createSky(assetManager, "Textures/Sky/Bright/BrightSky.dds", SkyFactory.EnvMapType.CubeMap);
//        Spatial sky = SkyFactory.createSky(assetManager, "Textures/Sky/Bright/BrightSky.dds", false);
//        rootNode.attachChild(sky);









        
        
        
    String path = "1/Scene/j3o2.j3o";
    BinaryExporter exporter = BinaryExporter.getInstance();
    assetManager.registerLocator("assets", FileLocator.class);
    File file = new File("assets/" + path.replace("Scene", "j3o"));
    try {
      exporter.save(rootNode, file);
    } catch (IOException ex) {
      ex.printStackTrace();
    }
    
    //super.stop()x`; // continue quitting the game
        


      

    }
        
//      @Override
//    public void stop() {
//    String path = "Models/Scenes/grassPlane/Scene/Scene.j3o";
//    assetManager.registerLocator("assets", FileLocator.class);
//    BinaryExporter exporter = BinaryExporter.getInstance();
//    Spatial model = (Spatial) assetManager.loadModel(path);
//    rootNode.attachChild(model);
//    File file = new File("assets/1/" + path.replace("Scene", "j3o"));
//    try {
//      exporter.save(model, file);
//    } catch (IOException ex) {
//      ex.printStackTrace();
//    }
//    super.stop(); // continue quitting the game
//  }
}