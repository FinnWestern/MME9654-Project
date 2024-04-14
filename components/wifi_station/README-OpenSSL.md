### Steps to create enterprise OpenSSL Certificates

1. Make directory tree
```
mkdir demoCA
mkdir demoCA/newcerts
mkdir demoCA/private
sh -c "echo '01' > ./demoCA/serial"
touch ./demoCA/index.txt
touch xpextensions
```
Add following lines in xpextensions file
```
[ xpclient_ext ]
extendedKeyUsage = 1.3.6.1.5.5.7.3.2

[ xpserver_ext ]
extendedKeyUsage = 1.3.6.1.5.5.7.3.1
```

2. Create ca.pem root certificate. This is the foundation of certificate validity
```
openssl req -new -x509 -keyout ca.key -out ca.pem
```

3. Generate rsa keys for client and server
```
openssl genrsa -out client.key 2048
openssl genrsa -out server.key 2048
```

4. Generate certificate signing req for both client and server
```
openssl req -new -key client.key -out client.csr
openssl req -new -key server.key -out server.csr
```

5. Create certs (.crt) for client nd server
```
openssl ca -batch -keyfile ca.key -cert ca.pem -in client.csr -key (password) -out client.crt -extensions xpserver_ext -extfile xpextensions
openssl ca -batch -keyfile ca.key -cert ca.pem -in server.csr -key (password) -out server.crt -extensions xpserver_ext -extfile xpextensions
```

6. Export .p12 files
```
openssl pkcs12 -export -out client.p12 -inkey client.key -in client.crt
openssl pkcs12 -export -out server.p12 -inkey server.key -in server.crt
```

7. Create .pem files
```
openssl pkcs12 -in client.p12 -out client.pem
openssl pkcs12 -in server.p12 -out server.pem
```
