# -
将预训练好的深度强化学习模型应用在真实机器人导航中。

先建图
<launch>
  <!-- 设置地图的配置文件 -->
  <arg name="map" default="map.yaml" />
  <!-- 运行地图服务器，并且加载设置的地图-->
  <node name="map_server" pkg="map_server" type="map_server" args="$(find nav)/map/$(arg map)"/>
  <!-- 启动AMCL节点 -->
  <include file="$(find nav)/launch/amcl.launch" />
  <!-- rviz -->
  <arg name="open_rviz" default="true"/>
  <group if="$(arg open_rviz)"> 
    <node pkg="rviz" type="rviz" name="rviz" required="true"
          args="-d $(find nav)/rviz/turtlebot3_navigation.rviz"/>
  </group>
  <!--turtlebot3-->
  <arg name="model" default="$(env TURTLEBOT3_MODEL)" doc="model type [burger, waffle, waffle_pi]"/>
  <include file="$(find turtlebot3_bringup)/launch/turtlebot3_remote.launch">
  <arg name="model" value="$(arg model)" />
  </include>
   
</launch>


## 启动自己的机器人底盘


## 启动上面的launch文件

## 设置机器人初始位置

## 发送目标位置
