import sys
sys.path.append('/home/mihawk/FoundationPose')


from ultralytics import YOLO
from estimater import *
from datareader import *
import argparse
import json
# model = YOLO("./runs/segment/train8/weights/best.pt")
# results = model("/home/mihawk/yolo/dataset1/images/0001.png", save=False, imgsz=320, conf=0.5)
 
# for result in results:
#     boxes = result.boxes
#     probs = result.probs
#     result.show()
    
# First import the library
import pyrealsense2 as rs
# Import Numpy for easy array manipulation
import numpy as np
# Import OpenCV for easy image rendering
import cv2
import socket


# video_name = 'output_video1.mp4'
# fourcc = cv2.VideoWriter_fourcc(*'mp4v')
# video = cv2.VideoWriter(video_name, fourcc, 30, (640, 480))

if __name__=='__main__':
    parser = argparse.ArgumentParser()
    code_dir = os.path.dirname(os.path.realpath(__file__))
    parser.add_argument('--mesh_file3', type=str, default='/home/mihawk/Cutie/dataset/Carton_Model_V3.obj')
    parser.add_argument('--mesh_file2', type=str, default='/home/mihawk/Cutie/dataset/Bottle_V1_cola2.obj')
    parser.add_argument('--mesh_file1', type=str, default='/home/mihawk/Cutie/dataset/Can_Model_V2.obj')
    parser.add_argument('--mesh_file4', type=str, default='/home/mihawk/Cutie/dataset/Can_Model_V2.obj')
    
    parser.add_argument('--test_scene_dir', type=str, default='/home/mihawk/Cutie/dataset1')
    parser.add_argument('--est_refine_iter', type=int, default=5)
    parser.add_argument('--track_refine_iter', type=int, default=2)
    parser.add_argument('--debug', type=int, default=1)
    parser.add_argument('--debug_dir', type=str, default=f'{code_dir}/debug')
    args = parser.parse_args()
    # video_writer = cv2.VideoWriter('output.avi', cv2.VideoWriter_fourcc(*'XVID'), 30, (640, 480))
    set_logging_format()
    set_seed(0)



    mesh1 = trimesh.load(args.mesh_file1,force='mesh')
    mesh1.vertices *= 2
    # mesh.vertices /=1000

    to_origin1, extents1 = trimesh.bounds.oriented_bounds(mesh1)
    bbox1 = np.stack([-extents1/2, extents1/2], axis=0).reshape(2,3)
    
    
    mesh2 = trimesh.load(args.mesh_file2,force='mesh')
    mesh2.vertices *= 2
    # mesh.vertices /=1000

    to_origin2, extents2 = trimesh.bounds.oriented_bounds(mesh2)
    bbox2 = np.stack([-extents2/2, extents2/2], axis=0).reshape(2,3)
    
    mesh3 = trimesh.load(args.mesh_file3,force='mesh')
    mesh3.vertices *= 2
    # mesh.vertices /=1000

    to_origin3, extents3 = trimesh.bounds.oriented_bounds(mesh1)
    bbox3 = np.stack([-extents3/2, extents3/2], axis=0).reshape(2,3)
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    
    host = '127.0.0.1'  
    port = 8888     
    sock.connect((host, port))
  
    find_bool = False
    first_bool = True
    number_bool = False
    debug = args.debug
    debug_dir = args.debug_dir
    os.system(f'rm -rf {debug_dir}/* && mkdir -p {debug_dir}/track_vis {debug_dir}/ob_in_cam')
  
    
    scorer = ScorePredictor()
    refiner = PoseRefinePredictor()
    glctx = dr.RasterizeCudaContext()
    
    
    est1 = FoundationPose(model_pts=mesh1.vertices, model_normals=mesh1.vertex_normals, mesh=mesh1, scorer=scorer, refiner=refiner, debug_dir=debug_dir, debug=debug, glctx=glctx)
    est2 = FoundationPose(model_pts=mesh2.vertices, model_normals=mesh2.vertex_normals, mesh=mesh2, scorer=scorer, refiner=refiner, debug_dir=debug_dir, debug=debug, glctx=glctx)
    est3 = FoundationPose(model_pts=mesh3.vertices, model_normals=mesh3.vertex_normals, mesh=mesh3, scorer=scorer, refiner=refiner, debug_dir=debug_dir, debug=debug, glctx=glctx)
    
    logging.info("estimator initialization done")

    reader = YcbineoatReader(video_dir=args.test_scene_dir, shorter_side=None, zfar=np.inf)


    # Create a pipeline
    pipeline = rs.pipeline()

    # Create a config and configure the pipeline to stream
    #  different resolutions of color and depth streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    profile = pipeline.start(config)

    # Getting the depth sensor's depth scale (see rs-align example for explanation)
    depth_sensor = profile.get_device().first_depth_sensor()
    depth_scale = depth_sensor.get_depth_scale()
    print("Depth Scale is: " , depth_scale)


    # Create an align object
    # rs.align allows us to perform alignment of depth frames to others frames
    # The "align_to" is the stream type to which we plan to align depth frames.
    align_to = rs.stream.color
    align = rs.align(align_to)
    #print camera intrinsics
    intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    # print(intr)
    # model = YOLO("/home/mihawk/yolo/runs/segment/train8/weights/best.pt")
    model = YOLO("/home/mihawk/GPHT/Aza_train13_19Feb(11m)/weights/best.pt")
    
    # model = YOLO("/home/mihawk/Documents/Aza_train07_22J/train2/weights/epoch120.pt")
    i = 0
    # Streaming loop
    output = np.zeros((480, 640), np.uint8)
    result_number = 0
    try:
        while True:
            # Get frameset of color and depth
            frames = pipeline.wait_for_frames()
            # frames.get_depth_frame() is a 640x360 depth image

            # Align the depth frame to color frame
            aligned_frames = align.process(frames)

            # Get aligned frames
            aligned_depth_frame = aligned_frames.get_depth_frame() # aligned_depth_frame is a 640x480 depth image
            color_frame = aligned_frames.get_color_frame()

            # Validate that both frames are valid
            if not aligned_depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(aligned_depth_frame.get_data())/1e3
            color_image = np.asanyarray(color_frame.get_data())
            
            # color_image = cv2.imread("/home/mihawk/Cutie/dataset2/rgb/0240.png")
            # depth_image = cv2.imread("/home/mihawk/Cutie/dataset2/depth/0240.png",-1)/1e3
            depth_image[(depth_image<0.001) | (depth_image>=np.inf)] = 0
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
            
            results = model(color_image, save=False, imgsz=640, conf=0.86)
            # logging.info(f'i:{i}')
            result_name = -1
            if len(results[0].boxes) != 0:
                find_bool = True
                result_name = int(results[0].boxes[0].cls.cpu().numpy()[0])
                print(f"results name: {result_name}")
                if result_number != results[0].boxes.shape[0]:
                    number_bool = False
                else:
                    number_bool = True
                result_number = results[0].boxes.shape[0]
                print(f"result_number: {result_number}")
            else:
                find_bool = False  
                first_bool = True
                 
            output = reader.get_mask(0).astype(bool)
            # if find_bool:
            
            if (find_bool and (first_bool or not number_bool)):
                # for boundingbox
                mask = results[0].boxes.xyxy[0]
                output = np.zeros((480, 640), np.uint8)
                cv2.rectangle(output, (int(mask[0]), int(mask[1])), (int(mask[2]), int(mask[3])), 255, -1)
                
                # for mask
                # mask = results[0].masks.xy[0]
                # pts = np.array(mask, np.int32)
                # pts = pts.reshape((-1, 1, 2))
                # output = np.zeros((480, 640), np.uint8)
                # cv2.fillPoly(output, [pts], 255)
                est = None
                
                if result_name == 0:
                    est = est1
                    bbox = bbox1
                    to_origin = to_origin1
                elif result_name == 1:
                    est = est2
                    bbox = bbox2
                    to_origin = to_origin2
                elif result_name == 2:
                    est = est3
                    bbox = bbox3
                    to_origin = to_origin3
                
                
                pose = est.register(K=reader.K, rgb=color_image, depth=depth_image, ob_mask=output, iteration=args.est_refine_iter)
                first_bool = False

            if find_bool and not first_bool:
                pose = est.track_one(rgb=color_image, depth=depth_image, K=reader.K, iteration=args.track_refine_iter)


                if debug>=1:
                    center_pose = pose@np.linalg.inv(to_origin)

                    center_pose[3][3] = result_name
                    json_data = json.dumps(center_pose.tolist())
                    sock.sendall(json_data.encode('utf-8'))
                    vis = draw_posed_3d_box(reader.K, img=color_image, ob_in_cam=center_pose, bbox=bbox)
                    vis = draw_xyz_axis(color_image, ob_in_cam=center_pose, scale=0.1, K=reader.K, thickness=3, transparency=0)
                    cv2.imshow('1', vis)
            
            
            
            
            for result in results: 
                im = result.plot(conf = False)
                # concat = np.hstack((im, output))
                cv2.imshow('image', im)
                # video.write(color_image)

            i += 1
            key = cv2.waitKey(1)
            if key & 0xFF == ord('q') or key == 27:
                cv2.destroyAllWindows()
                break
            if key & 0xFF == ord('r'):
                first_bool = True
    finally:
        pipeline.stop()
        s.close()
        # video.release()
        cv2.destroyAllWindows()
        print("finish")
                    