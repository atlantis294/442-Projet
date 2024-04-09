x = 0
y = 0
largeur = 480
hauteur = 272

file=open("mapENS.bmp" ,'rb')
data=file.read()
file.close()


offset=data[10]
image=data[:offset]
print(image)
h=700
l=1707

# image[18]=0xE0
# image[19]=0x01
# image[20]=0x00
# image[21]=0x00
# image[22]=0x10
# image[23]=0x01
# image[24]=0x00
# image[25]=0x00
image = bytearray(image)  # Convertir en bytearray pour pouvoir modifier les octets
image[18:22] = bytearray([0xE0, 0x01, 0x00, 0x00])  # Modifier les octets de l'en-tête
image[22:26] = bytearray([0x10, 0x01, 0x00, 0x00])
index=137
for j in range(y,y+hauteur):
    k=offset+2*(j*(l+1)+x)
    for i in range(2*largeur):
        index=k+i
        image.append(data[index])        

file_output=open('Sortie.bmp','wb')
file_output.write(image)
file_output.close()

print("Fin")





