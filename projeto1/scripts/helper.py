import cv2
def encontrar_maior_contorno(segmentado):
        """Não mude ou renomeie esta função
        Entrada:
            segmentado - imagem em preto e branco
        Saída:
            contorno - maior contorno obtido (APENAS este contorno)
        """
    
        contornos, arvore = cv2.findContours(segmentado.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
        contorno = None
        maior_area = 0
        for c in contornos:
            area = cv2.contourArea(c)
            if area > maior_area:
                maior_area = area
                contorno = c
        return contorno