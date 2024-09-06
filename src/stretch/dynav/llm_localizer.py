from io import BytesIO
from concurrent.futures import ThreadPoolExecutor, as_completed
import time
import os

import torch
import numpy as np
from PIL import Image

from typing import Optional
from torch import Tensor

from stretch.utils.point_cloud_torch import unproject_masked_depth_to_xyz_coordinates
from stretch.dynav.mapping_utils import VoxelizedPointcloud

from transformers import AutoProcessor
from transformers import Owlv2ForObjectDetection

import google.generativeai as genai
from openai import OpenAI
import base64
genai.configure(api_key=os.getenv('GOOGLE_API_KEY'))
generation_config = genai.GenerationConfig(temperature=0)
safety_settings = [
    {
        "category": "HARM_CATEGORY_DANGEROUS",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_HARASSMENT",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_HATE_SPEECH",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_SEXUALLY_EXPLICIT",
        "threshold": "BLOCK_NONE",
    },
    {
        "category": "HARM_CATEGORY_DANGEROUS_CONTENT",
        "threshold": "BLOCK_NONE",
    },
]
class LLM_Localizer():
    def __init__(self, voxel_map_wrapper = None, exist_model = 'gpt-4o', loc_model = 'owlv2', device = 'cuda'):
        self.voxel_map_wrapper = voxel_map_wrapper
        self.device = device
        self.voxel_pcd = VoxelizedPointcloud(voxel_size=0.2).to(self.device)
        # self.exist_model = YOLOWorld("yolov8l-worldv2.pt")
        self.existence_checking_model = exist_model
        if exist_model == 'gpt-4o':
            print('WE ARE USING OPENAI GPT4o')
            self.gpt_client = OpenAI(api_key=os.getenv('OPENAI_API_KEY'))
        elif exist_model == 'gemini-1.5-pro':
            print('WE ARE USING GEMINI 1.5 PRO')
            
        elif exist_model == 'gemini-1.5-flash':
            print('WE ARE USING GEMINI 1.5 FLASH')
        else:
            print('YOU ARE USING NOTHING!')
        self.location_checking_model = loc_model
        if loc_model == 'owlv2':
            self.exist_processor = AutoProcessor.from_pretrained("google/owlv2-base-patch16-ensemble")
            self.exist_model = Owlv2ForObjectDetection.from_pretrained("google/owlv2-base-patch16-ensemble").to(self.device)
            print('WE ARE USING OWLV2 FOR LOCALIZATION!')
        else:
            print('YOU ARE USING VOXEL MAP FOR LOCALIZATION!')
        
    def add(self,
        points: Tensor,
        features: Optional[Tensor],
        rgb: Optional[Tensor],
        weights: Optional[Tensor] = None,
        obs_count: Optional[Tensor] = None,
    ):
        points = points.to(self.device)
        if features is not None:
            features = features.to(self.device)
        if rgb is not None:
            rgb = rgb.to(self.device)
        if weights is not None:
            weights = weights.to(self.device)
        self.voxel_pcd.add(points = points, 
                        features = features,
                        rgb = rgb,
                        weights = weights,
                        obs_count = obs_count)

    def owl_locater(self, A, encoded_image, timestamps_lst):
        for i in timestamps_lst:
            image_info = encoded_image[i][-1]
            image = image_info['image']
            box = None
                
            inputs = self.exist_processor(text=A, images=image, return_tensors="pt").to(self.device)

            with torch.no_grad():
                outputs = self.exist_model(**inputs)

            target_sizes = torch.tensor([image.size[::-1]])
            results = self.exist_processor.post_process_object_detection(outputs=outputs, target_sizes=target_sizes, threshold=0.2)[0]

            if len(results["scores"]) > 0:
                cur_score = torch.max(results["scores"]).item()
                max_score_index = torch.argmax(results["scores"])
                box = results["boxes"][max_score_index].tolist()
            if box is not None:
                xmin, ymin, xmax, ymax = map(int, box)
                mask = np.zeros(image_info['depth'].shape, dtype=np.uint8)
                mask[ymin:ymax, xmin:xmax] = 255
                xyz = image_info['xyz']        
                masked_xyz = xyz[mask.flatten() > 0]
                centroid = np.stack([torch.mean(masked_xyz[:, 0]), torch.mean(masked_xyz[:, 1]), torch.mean(masked_xyz[:, 2])]).T
                debug_text = '#### - Obejct is detected in observations where instance' + str(i + 1) + ' comes from. **😃** Directly navigate to it.\n'
                return centroid, debug_text, i, masked_xyz
        debug_text = '#### - All instances are not the target! Maybe target object has not been observed yet. **😭**\n'
        return None, debug_text, None, None
    
    def gpt_chunk(self, chunk, sys_prompt, user_prompt):
        for i in range(10):
            try:
                response = self.gpt_client.chat.completions.create(
                    model=self.existence_checking_model,
                    messages=[
                        {"role": "system", "content": sys_prompt},
                        {"role": "user", "content": user_prompt},
                        {"role": "user", "content": chunk}
                    ],
                    temperature=0.0,
                )                
                timestamps = response.choices[0].message.content
                if 'None' in timestamps:
                    return None
                else:
                    return list(map(int, timestamps.replace(' ', '').split(':')[1].split(',')))
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(15)
        return "Execution Failed"

    def gemini_chunk(self, chunk, sys_prompt, user_prompt):
        if self.existence_checking_model == 'gemini-1.5-pro':
            model_name="models/gemini-1.5-pro-exp-0827"
        elif self.existence_checking_model == 'gemini-1.5-flash':
            model_name="models/gemini-1.5-flash-exp-0827"

        for i in range(3):
            try:
                model = genai.GenerativeModel(model_name=model_name, system_instruction=sys_prompt)
                timestamps = model.generate_content(chunk + [user_prompt], generation_config=generation_config, safety_settings=safety_settings).text
                timestamps = timestamps.split('\n')[0]
                if 'None' in timestamps:
                    return None
                else:
                    return list(map(int, timestamps.replace(' ', '').split(':')[1].split(',')))
            except Exception as e:
                print(f"Error: {e}")
                time.sleep(10)
        return "Execution Failed"



    def llm_locator(self, A, encoded_image, process_chunk, context_length = 30):
        timestamps_lst = []

        sys_prompt = f"""
        For each object query provided, list at most 10 timestamps that the object is most clearly shown. If the object does not appear, simply output the object name and the word "None" for the timestamp. Avoid any unnecessary explanations or additional formatting.
        
        Example:
        Input:
        cat

        Output: 
        cat: 1,4,6,9

        Input: 
        car

        Output:
        car: None
        """

        user_prompt = f"""The object you need to find is {A}"""
        if 'gpt' in self.existence_checking_model:
            content = [item for sublist in list(encoded_image.values()) for item in sublist[:2]][-100:] # adjust to [-60:] for taking only the last 30 and have faster speed
        elif 'gemini' in self.existence_checking_model:
            content = [item for sublist in list(encoded_image.values()) for item in sublist[0]][-100:]
        content_chunks = [content[i:i + 2 * context_length] for i in range(0, len(content), 2 * context_length)]
        
        with ThreadPoolExecutor(max_workers=2) as executor:
            future_to_chunk = {executor.submit(process_chunk, chunk, sys_prompt, user_prompt): chunk for chunk in content_chunks}
            
            for future in as_completed(future_to_chunk):
                chunk = future_to_chunk[future]
                try:
                    result = future.result()
                    if result:
                        timestamps_lst.extend(result)    
                except Exception as e:
                    print(f"Exception occurred: {e}")
        timestamps_lst = sorted(timestamps_lst, reverse=True)
        # print(A, timestamps_lst)
        return self.owl_locater(A, encoded_image, timestamps_lst)

    def localize_A(self, A, debug = True, return_debug = False, count_threshold = 5):
        encoded_image = {}

        counts = torch.bincount(self.voxel_map_wrapper.voxel_pcd._obs_counts)
        cur_obs = max(self.voxel_map_wrapper.voxel_pcd._obs_counts)
        filtered_obs = (counts > count_threshold).nonzero(as_tuple=True)[0].tolist()
        # filtered_obs = sorted(set(filtered_obs + [i for i in range(cur_obs-10, cur_obs+1)]))
        filtered_obs = sorted(filtered_obs)

        # filtered_obs = (counts <= count_threshold).nonzero(as_tuple=True)[0].tolist()
        # filtered_obs = [obs for obs in filtered_obs if (cur_obs - obs) >= 10]

        if 'gemini' in self.existence_checking_model:
            process_chunk = self.gemini_chunk
            context_length = 100
        elif 'gpt' in self.existence_checking_model:
            process_chunk = self.gpt_chunk
            context_length = 30

        for obs_id in filtered_obs: 
            rgb = self.voxel_map_wrapper.observations[obs_id - 1].rgb.numpy()
            depth = self.voxel_map_wrapper.observations[obs_id - 1].depth
            camera_pose = self.voxel_map_wrapper.observations[obs_id - 1].camera_pose
            camera_K = self.voxel_map_wrapper.observations[obs_id - 1].camera_K

            full_world_xyz = unproject_masked_depth_to_xyz_coordinates(  # Batchable!
                depth=depth.unsqueeze(0).unsqueeze(1),
                pose=camera_pose.unsqueeze(0),
                inv_intrinsics=torch.linalg.inv(camera_K[:3, :3]).unsqueeze(0),
            )
            depth = depth.numpy()
            rgb[depth > 2.5] = [0, 0, 0]
            image = Image.fromarray(rgb.astype(np.uint8), mode='RGB')
            if 'gemini' in self.existence_checking_model:
                encoded_image[obs_id] = [[f"Following is the image took on timestep {obs_id}: ", image], {'image':image, 'xyz':full_world_xyz, 'depth':depth}]
            elif 'gpt' in self.existence_checking_model:
                buffered = BytesIO()
                image.save(buffered, format="PNG")
                img_bytes = buffered.getvalue()
                base64_encoded = base64.b64encode(img_bytes).decode('utf-8')
                encoded_image[obs_id] = [{"type": "text", "text": f"Following is the image took on timestep {obs_id}"},
                    {"type": "image_url", "image_url": {
                        "url": f"data:image/png;base64,{base64_encoded}"}
                    }, {'image':image, 'xyz':full_world_xyz, 'depth':depth}]
        target_point, debug_text, obs, point = self.llm_locator(A, encoded_image, process_chunk, context_length)
        if not debug:
            return target_point
        elif not return_debug:
            return target_point, debug_text
        else:
            return target_point, debug_text, obs, point