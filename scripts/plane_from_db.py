#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd
from db_interface import DataBase
from tf_utils import Tf as TF
import rosparam
import rospy
DIS = 0.05

# visualization
import roslib; roslib.load_manifest('visualization_marker_tutorials')
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import matplotlib.cm as cm
color = [cm.gist_rainbow(i / float(1000))for i in range(1000)]


def primary_function(x1, y1, x2, y2):
    """
    a = (y2- y1) / (x2 -x1)
    b = y1 - ax1
    Return
    y = ax + b
    ----------
    a: float
    b: float
    """
    a = (y2 -y1) / ((x2 -x1))
    b = y1 - a * x1

    return [a, b]

class Plane(object):

    u"""
    物体検出・認識に関する処理を扱うオーダークラス
    """
    def __init__(self,):
        u"""
        Paramaters
        -----
        robot : class instance
            hsrb_interface robot.py Robot
        """
        db_name = rosparam.get_param("/db_param/dbname")
        host = rosparam.get_param("/db_param/host")
        user = rosparam.get_param("/db_param/user")
        password = rosparam.get_param("/db_param/password")
        table = "plane_info"
        self.db = DataBase(db_name, host, user, password, table)
        self._tf_utils = TF()
        self.pub_marker = rospy.Publisher("visualization_objects", MarkerArray, queue_size=1)
        self.parent_frame = "map"

        rospy.loginfo("object class create success")

    def _visulaize_marker(self, sql_data):
        """
        可視化用
        未実装
        """
        return False
        # rospy.sleep(0.1)
        m = MarkerArray()
        marker = Marker()
        marker.action = 3 # alls marker deleate
        m.markers.append(marker)
        self.pub_marker.publish(m)
        markerArray = MarkerArray()
        marker = Marker()
        for i, _ in sql_data.iterrows():
            marker = Marker()
            objects_pos = list(sql_data.ix[i, ["center_x", "center_y", "center_z"]])#, "generic_id_0", "generic_score_0"]])
            marker.id = i #"object_{0}".format(i)
            marker.header.frame_id = "/map"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.scale.x = 0.08 # objects_pos[4] / 10.0
            marker.scale.y = 0.08 # objects_pos[4] / 10.0
            marker.scale.z = 0.08 # objects_pos[4] / 10.0
            marker.color.a = 1 #objects_pos[4]
            marker.color.r = color[objects_pos[3]][0]
            marker.color.g = color[objects_pos[3]][1]
            marker.color.b = color[objects_pos[3]][2]
            marker.pose.orientation.w = 1.0
            marker.pose.position.x = objects_pos[0]
            marker.pose.position.y = objects_pos[1]
            marker.pose.position.z = objects_pos[2]
            markerArray.markers.append(marker)
        self.pub_marker.publish(markerArray)

    def _extraction_threshold_area(self, data_frame, threshold_area):
        u"""
        | データフレームの座標を閾値以上の距離にあるものを削除し、同一と思われるものを排除するメソッド
        |
        Params
        -----
        data_frame : DataFrame
            座標点の削減を行うためのDataFrame
        threshold_area : float
            データフレーム内の平面の面積の閾値
        Returns
        -----
        data_frame : DataFrame
            座標点の削減を行ったDataFrame
        """
        rospy.loginfo(data_frame)
        data_frame = data_frame.loc[(((data_frame.lowerleft_x *data_frame.lowerright_y)-(data_frame.lowerright_x*data_frame.lowerleft_y)+
                                     (data_frame.lowerright_x *data_frame.upperright_y)-(data_frame.upperright_x*data_frame.lowerright_y)+
                                     (data_frame.upperright_x *data_frame.upperleft_y)-(data_frame.upperleft_x*data_frame.upperright_y)+
                                     (data_frame.upperleft_x *data_frame.lowerleft_y)-(data_frame.lowerleft_x*data_frame.upperleft_y))/2.0 > threshold_area)]

        return data_frame

    def _calc_area_point(self, minx, maxx, miny, maxy, base_tf):
        u"""
        | データベースから入手した座標をbase_tfから見たエリアで区切る為に
        | エリアの必要な点の座標変換を行う
        Params
        -----
        minx : float
            base_tfから見たxの最小値、base_footprint基準ではどれだけ手前かを表す(負だと背後)
        maxx : float
            base_tfから見たxの最大値、base_footprint基準ではどれだけ奥かを表す
        miny : float
            base_tfから見たyの最小値、base_footprint基準ではどれだけ右かを表す（負右正左）
        maxy : float
            base_tfから見たyの最大値、base_footprint基準ではどれだけ左かを表す（負右正左）
        base_tf : string
            エリアの基準と成るtf
        Returns
        -----
        min_x : list of float
            base_tf基準の座標点をmap基準に変換したもの、その中におけるx最小となる[x,y]
        min_y : list of float
            base_tf基準の座標点をmap基準に変換したもの、その中におけるy最小となる[x,y]
        max_y : list of float
            base_tf基準の座標点をmap基準に変換したもの、その中におけるy最大となる[x,y]
        max_x : list of float
            base_tf基準の座標点をmap基準に変換したもの、その中におけるx最大となる[x,y]
        """
        xy_list = []
        while not rospy.is_shutdown():
            xyz= self._tf_utils.calcpos_base2child(minx, miny, 0.0, base_tf, "map")
            if xyz != []:
                break
            rospy.logwarn("can not look tf")
        xy_list.append(xyz)

        while not rospy.is_shutdown():
            xyz= self._tf_utils.calcpos_base2child(minx, maxy, 0.0, base_tf, "map")
            if xyz != []:
                break
            rospy.logwarn("can not look tf")
        xy_list.append(xyz)

        while not rospy.is_shutdown():
            xyz= self._tf_utils.calcpos_base2child(maxx, miny, 0.0, base_tf, "map")
            if xyz != []:
                break
            rospy.logwarn("can not look tf")
        xy_list.append(xyz)

        while not rospy.is_shutdown():
            xyz= self._tf_utils.calcpos_base2child(maxx, maxy, 0.0, base_tf, "map")
            if xyz != []:
                break
            rospy.logwarn("can not look tf")
        xy_list.append(xyz)
        array = np.array(xy_list)
        max_x = array[array[:,0].argmax()]
        min_x = array[array[:,0].argmin()]
        max_y = array[array[:,1].argmax()]
        min_y = array[array[:,1].argmin()]

        return min_x, min_y, max_y, max_x

    def _near_point_deleate(self, data_frame, score_type=True):
        u"""
        | データフレームの座標を閾値以上の距離にあるものを削除し、同一と思われるものを排除するメソッド
        |
        Params
        -----
        data_frame : DataFrame
            座標点の削減を行うためのDataFrame
        score_type : bool
            削除するにあたりscoreによるソートを行うが、Trueならば特定物体認識、Falseならば一般物体認識のスコアを用いる
        Returns
        -----
        data_frame : DataFrame
            座標点の削減を行ったDataFrame
        """
        data_frame = data_frame.sort_values(by="ros_timestamp",ascending=False)
        N = len(data_frame)
        df = data_frame[0:1]
        x = df.center_x.values[-1]
        y = df.center_y.values[-1]
        z = df.center_z.values[-1]
        for i in range(N):
            data_frame = data_frame.loc[(((data_frame.center_x -x)**2 + (data_frame.center_y -y) **2 +(data_frame.center_z -z) **2 ) > DIS**2 )]
            if len(data_frame) == 0:
                break
            ddf = data_frame[0:1]
            df = pd.concat([df, ddf])
            if len(data_frame) == 1:
                break
            x = ddf.center_x.values[-1]
            y = ddf.center_y.values[-1]
            z = ddf.center_z.values[-1]
        df = df.sort_values(by="ros_timestamp",ascending=False)
        self._visulaize_marker(df)
        return df

    def _select_data_from_object_position(self, data_frame, minx, maxx, miny, maxy, minz, maxz, base_tf):
        u"""
        | DataFrameからエリアで情報を取得するメソッド
        Params
        -----
        data_frame : DataFrame
            座標点の検索を行うためのDataFrame
        minx : float
            base_tfから見たxの最小値、base_footprint基準ではどれだけ手前かを表す(負だと背後)
        maxx : float
            base_tfから見たxの最大値、base_footprint基準ではどれだけ奥かを表す
        miny : float
            base_tfから見たyの最小値、base_footprint基準ではどれだけ右かを表す（負右正左）
        maxy : float
            base_tfから見たyの最大値、base_footprint基準ではどれだけ左かを表す（負右正左）
        minz : float
            mapから見たzの最小値、高さをあらわす
        maxz : float
            mapから見たzの最大値、高さをあらわす
        base_tf : string
            エリアの基準と成るtf
        Returns
        -----
        data_frame :
            データベースから獲得した情報
        """
##  ｚで切る
        data_frame = data_frame.loc[(data_frame.center_z >= minz) & (data_frame.center_z <= maxz)]

##  エリアで切る


        min_x, min_y, max_y, max_x = self._calc_area_point(minx, maxx, miny, maxy, base_tf)

        mix2miy = primary_function(min_x[0], min_x[1], min_y[0], min_y[1])
        mix2may = primary_function(min_x[0], min_x[1], max_y[0], max_y[1])
        miy2max = primary_function(min_y[0], min_y[1], max_x[0], max_x[1])
        may2max = primary_function(max_y[0], max_y[1], max_x[0], max_x[1])


        data_frame = data_frame.loc[((mix2miy[0] * data_frame.center_x + mix2miy[1] - data_frame.center_y) <= 0.0) ]
        rospy.loginfo("select1")
        rospy.loginfo(len(data_frame))
        data_frame = data_frame.loc[((mix2may[0] * data_frame.center_x + mix2may[1] - data_frame.center_y) >= 0.0) ]
        rospy.loginfo("select2")
        rospy.loginfo(len(data_frame))
        data_frame = data_frame.loc[((miy2max[0] * data_frame.center_x + miy2max[1] - data_frame.center_y) <= 0.0) ]
        rospy.loginfo("select3")
        rospy.loginfo(len(data_frame))
        data = data_frame.loc[((may2max[0] * data_frame.center_x + may2max[1] - data_frame.center_y) >= 0.0) ]
        rospy.loginfo("select4")
        rospy.loginfo(len(data))


#        data = data_frame.loc[((mix2miy[0] * data_frame.center_x + mix2miy[1] - data_frame.center_y) <= 0.0) &
#                              ((mix2may[0] * data_frame.center_x + mix2may[1] - data_frame.center_y) >= 0.0) &
#                              ((miy2max[0] * data_frame.center_x + miy2max[1] - data_frame.center_y) <= 0.0) &
#                              ((may2max[0] * data_frame.center_x + may2max[1] - data_frame.center_y) >= 0.0) ]
        return data

    def _get_plane_info_all(self,):
        u"""
        | DataBaseからすべての情報を取得するメソッド
        Returns
        -----
        data_frame :
            データベースから獲得した情報
        """
        data_frame = self.db._get_objectdata_from_time()
        return data_frame

    def _get_plane_info_from_time(self, tstart=5, tend=0):
        u"""
        | DataBaseから指定した時間の情報を取得するメソッド
        Params
        -----
        tstart : rospy.time、 default rospy.Time(0)
            情報を引き出すための開始時間
        tend : rospy.time、 default rospy.Time.now()
            情報を引き出すための終了時間
        Returns
        -----
        data_frame :
            データベースから獲得した情報
        """
        ts = rospy.Time.now() - rospy.Duration(tstart)
        ts = ts.to_nsec()
        te = rospy.Time.now().to_nsec()
        data_frame = self.db.get_objects_info_filtered_time(ts, te)
        return data_frame

    def _get_plane_info_latest(self):
        u"""
        | 最新の顔情報をすべて取得するメソッド
        """
        data_frame = self.db.get_objects_info_latest()
        return data_frame


    def get_plane_num_latest(self, threshold_area):
        """
        見つけた顔候補数を返すメソッド
        Params
        -----
        threshold_score : float
            スコアの閾値
        score_type : bool
            Trueならば特定物体認識、Falseならば一般物体認識のスコアを用いる
        Returns
        -----
        num_faces : int
            見つけた物体候補の数
        """
##     スコアによる物体の閾値が必要？
        data_frame = self._get_plane_info_latest()
        if len(data_frame) == 0:
            return 0
        data_frame = self._extraction_threshold_area(data_frame, threshold_area)
        if len(data_frame) == 0:
            return 0
        data_frame = self._near_point_deleate(data_frame)
        num_plane = len(data_frame)
        rospy.loginfo("Num_Plane:{}".format(num_plane))
        return num_plane

    def get_plane_num_from_time(self, tstart, tend, threshold_area, score_type):
        """
        見つけた顔候補数を返すメソッド
        Params
        -----
        tstart : rospy.time、 default rospy.Time(0)
            情報を引き出すための開始時間
        tend : rospy.time、 default rospy.Time.now()
            情報を引き出すための終了時間
        threshold_score : float
            スコアの閾値
        score_type : bool
            Trueならば特定物体認識、Falseならば一般物体認識のスコアを用いる
        Returns
        -----
        num_faces : int
            見つけた物体候補の数
        """
##     スコアによる物体の閾値が必要？
        data_frame = self._get_plane_info_from_time(tstart, tend)
        if len(data_frame) == 0:
            return 0
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data_frame))
        data_frame = self._extraction_threshold_area(data_frame, threshold_area)
        if len(data_frame) == 0:
            return 0
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data_frame))
        data_frame = self._near_point_deleate(data_frame)
        num_plane = len(data_frame)
        rospy.loginfo("Num_Plane:{}".format(num_plane))
        return num_plane

    def get_plane_info_filterd_time(self, tstart, tend, threshold_area, wear_flag=True):
        """
        特定顔認識のスコアが一定以上のものかつ
        指定された時刻に発見された物体の位置とidを返すメソッド
        どうやら現在は顔のスコアがないらしい
        Params
        -----
        tstart : rospy.time、 default rospy.Time(0)
            情報を引き出すための開始時間
        tend : rospy.time、 default rospy.Time.now()
            情報を引き出すための終了時間
        threshold_area : float
            スコアの閾値
        score_type : bool
            Trueならば特定物体認識、Falseならば一般物体認識のスコアを用いる
        Returns
        -----
        plane_pos : list of float
            見つけた平面位置
        """
        data = self._get_plane_info_from_time(tstart, tend)
        if len(data) == 0:
            return []
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data))
        data = self._extraction_threshold_area(data, threshold_area)
        if len(data) == 0:
            return []
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data))
        if wear_flag:
            data = self._near_point_deleate(data)
        data = data.sort_values(by="ros_timestamp",ascending=False)
        N = len(data)
        plane_pos = []
        for i in range(N):
            plane_pos.append([data.center_x.values[i], data.center_y.values[i], data.center_z.values[i]])
        return plane_pos

    def get_plane_info_filterd_time_and_area(self, tstart, tend,  threshold_area, minx, maxx, miny, maxy, minz, maxz, base_tf, score_type=True, wear_flag=True):
        """
        指定された範囲内で発見されたものの位置とIDを返すメソッド
        Params
        -----
        tstart : rospy.time、 default rospy.Time(0)
            情報を引き出すための開始時間
        tend : rospy.time、 default rospy.Time.now()
            情報を引き出すための終了時間
        threshold_area : float
            スコアの閾値
        minx : float
            base_tfから見たxの最小値、base_footprint基準ではどれだけ手前かを表す(負だと背後)
        maxx : float
            base_tfから見たxの最大値、base_footprint基準ではどれだけ奥かを表す
        miny : float
            base_tfから見たyの最小値、base_footprint基準ではどれだけ右かを表す（負右正左）
        maxy : float
            base_tfから見たyの最大値、base_footprint基準ではどれだけ左かを表す（負右正左）
        minz : float
            mapから見たzの最小値、高さをあらわす
        maxz : float
            mapから見たzの最大値、高さをあらわす
        base_tf : string
            エリアの基準と成るtf
        score_type : bool
            Trueならば特定物体認識、Falseならば一般物体認識のスコアを用いる
        Returns
        -----
        obj_pos : list of float
            見つけた物体位置

        """
        data = self._get_plane_info_from_time(tstart, tend)
        if len(data) == 0:
            return []
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data))
        data = self._select_data_from_object_position(data,  minx, maxx,  miny, maxy, minz, maxz, base_tf)
        if len(data) == 0:
            return []
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data))
        data = self._extraction_threshold_area(data, threshold_area)
        if len(data) == 0:
            return []
        else:
            rospy.loginfo("plane")
            rospy.loginfo(len(data))
        if wear_flag:
            data = self._near_point_deleate(data, score_type)
        data = data.sort_values(by="ros_timestamp",ascending=False)
        N = len(data)
        plane_pos = []
        for i in range(N):
            plane_pos.append([data.center_x.values[i], data.center_y.values[i], data.center_z.values[i]])
        return plane_pos

if __name__ == '__main__':
    rospy.init_node("vision_test")
    obj = Plane()
    while not rospy.is_shutdown():
        obj.get_object_num_from_time(10,0,-1,False)
