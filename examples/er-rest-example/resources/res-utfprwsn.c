/*
 * Copyright (c) 2013, Institute for Pervasive Computing, ETH Zurich
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 */

/**
 * \file
 *      Example resource
 * \author
 *      Matthias Kovatsch <kovatsch@inf.ethz.ch>
 */

#include <stdlib.h>
#include <string.h>
#include "rest-engine.h"
#include "er-coap.h"

static uint16_t buffernumber1[60];
static uint16_t buffernumber2[60];
static uint16_t buffernumber3[60];
static uint16_t buffernumber4[60];
static uint16_t tamanho_do_buffer1;
static uint16_t tamanho_do_buffer2;
static uint16_t tamanho_do_buffer3;
static uint16_t tamanho_do_buffer4;
static uint16_t posicao_buffer;

static void res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);
static void res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset);

/*
 * A handler function named [resource name]_handler must be implemented for each RESOURCE.
 * A buffer for the response payload is provided through the buffer pointer. Simple resources can ignore
 * preferred_size and offset, but must respect the REST_MAX_CHUNK_SIZE limit for the buffer.
 * If a smaller block size is requested for CoAP, the REST framework automatically splits the data.
 */
RESOURCE(res_utfprwsn,
         "title=\"UTFPRWSN\";rt=\"Text\"",
         res_get_handler,
         res_post_put_handler,
         res_post_put_handler,
         NULL);

static char message[REST_MAX_CHUNK_SIZE] = "skamsfrevrest";
//static uint32_t length= 13;
const char *len = NULL;
static int32_t length;
static void
res_get_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{

    uint32_t i;
    uint8_t etag=0;

    //configura o tipo de conteudo da mensagem
    REST.set_header_content_type(response, REST.type.TEXT_PLAIN);
    /* The query string can be retrieved by rest_get_query() or parsed for its key-value pairs. */
    if(REST.get_query_variable(request, "len", &len)) {
         length = atoi(len);
         }

    //etag eh uma propriedade que eh utilizada pelos servidores de cache para saber se uma mensagem mudou
    //duas mensagens com o mesmo valor devem ter o mesmo etag
    if (length == 1){
    for(i=0;i<tamanho_do_buffer1;i++)
    {
        //neste caso utilizamos um checksum simples como etag, mas o usuario pode usar o que quiser
        etag += buffernumber1[i];
    }
    REST.set_header_etag(response, (uint8_t *)&etag, buffernumber1);

       //configura o payload a ser retornado
       REST.set_response_payload(response, buffernumber1, tamanho_do_buffer1);
    }
    if (length == 2){
       for(i=0;i<tamanho_do_buffer2;i++)
       {
           //neste caso utilizamos um checksum simples como etag, mas o usuario pode usar o que quiser
           etag += buffernumber2[i];
       }
       REST.set_header_etag(response, (uint8_t *)&etag, 1);

          //configura o payload a ser retornado
          REST.set_response_payload(response, buffernumber2, tamanho_do_buffer2);
       }
    if (length == 3){
       for(i=0;i<tamanho_do_buffer3;i++)
       {
           //neste caso utilizamos um checksum simples como etag, mas o usuario pode usar o que quiser
           etag += buffernumber3[i];
       }
       REST.set_header_etag(response, (uint8_t *)&etag, 1);

          //configura o payload a ser retornado
          REST.set_response_payload(response, buffernumber3, tamanho_do_buffer3);
       }
    if (length == 4){
         for(i=0;i<tamanho_do_buffer4;i++)
         {
             //neste caso utilizamos um checksum simples como etag, mas o usuario pode usar o que quiser
             etag += buffernumber4[i];
         }
         REST.set_header_etag(response, (uint8_t *)&etag, 1);

            //configura o payload a ser retornado
            REST.set_response_payload(response, buffernumber4, tamanho_do_buffer4);
         }
    //REST.set_header_etag(response, (uint8_t *)&etag, 1);

    //configura o payload a ser retornado
    //REST.set_response_payload(response, buffernumber1, tamanho_do_buffer1);
}


//static void
//res_post_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
//{
//  leds_toggle(LEDS_RED);
//}

static void
res_post_put_handler(void *request, void *response, uint8_t *buffer, uint16_t preferred_size, int32_t *offset)
{
    //converte o payload recebido por PUT em um pacote CoAP
    coap_packet_t *const coap_req = (coap_packet_t *)request;
    uint8_t buffer_ptr = 0;

    if(REST.get_query_variable(request, "len", &len)) {
           length = atoi(len);
           posicao_buffer = length;
      }

    //verifica se o payload enviado nao eh muito grande para a requisicao
    if(coap_req->payload_len > REST_MAX_CHUNK_SIZE)
    {
        //caso for muito grande, simplesmente configura a resposta como BAD_REQUEST e retorna
        REST.set_response_status(response, REST.status.BAD_REQUEST);
        return;
    }
    else
    {
        if (length == 1){

            memcpy((void*)buffernumber1, (void*)coap_req->payload, coap_req->payload_len);
            tamanho_do_buffer1 = coap_req->payload_len;
            }
        else if(length == 2){

            memcpy((void*)buffernumber2, (void*)coap_req->payload, coap_req->payload_len);
            tamanho_do_buffer2 = coap_req->payload_len;
            }
        else if (length == 3){

            memcpy((void*)buffernumber3, (void*)coap_req->payload, coap_req->payload_len);
            tamanho_do_buffer3 = coap_req->payload_len;
            }
        else if (length == 4){

            memcpy((void*)buffernumber4, (void*)coap_req->payload, coap_req->payload_len);
            tamanho_do_buffer4 = coap_req->payload_len;
            }

        //caso contrario, copia a mensagem enviada para o buffer criado

        //salva tambem o tamanho da mensagem recebida (para uso futuro)
        //tamanho_do_buffer = coap_req->payload_len;
    }
}


