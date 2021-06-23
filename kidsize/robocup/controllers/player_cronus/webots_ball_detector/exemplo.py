import numpy as np
from dodo_detector.detection import TFObjectDetectorV2
from PIL import Image

import tensorflow as tf

##############################################################################
# Necessário se houver o seguinte erro
# Could not create cudnn handle: CUDNN_STATUS_INTERNAL_ERROR
##############################################################################
gpus = tf.config.list_physical_devices('GPU')
if gpus:
   try:
      # Currently, memory growth needs to be the same across GPUs
      for gpu in gpus:
         tf.config.experimental.set_memory_growth(gpu, True)
      logical_gpus = tf.config.experimental.list_logical_devices('GPU')
      print(len(gpus), "Physical GPUs,", len(logical_gpus), "Logical GPUs")
   except RuntimeError as e:
      # Memory growth must be set before GPUs have been initialized
      print(e)
##############################################################################

# carrega modelo da pasta 'saved model' e rótulos das classes do arquivo 'label_map.pbtxt'
# leva um tempinho
detector = TFObjectDetectorV2('exported/saved_model/', 'label_map.pbtxt')
# abre uma imagem JPG (precisa ter 3 canais)
im = np.array((Image.open('im.jpg').convert('RGB')))   #im.jpg -> image recebida e tratada
# newim é a imagem com os retângulos marcados
# results é um dicionário onde as chaves são as classes e os calores são listas de listas, com os retângulos dos objetos detectados
# a primeira detecção é demorada, as demais não
newim, results = detector.from_image(im)
