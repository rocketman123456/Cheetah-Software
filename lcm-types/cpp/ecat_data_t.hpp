/** THIS IS AN AUTOMATICALLY GENERATED FILE.  DO NOT MODIFY
 * BY HAND!!
 *
 * Generated by lcm-gen
 **/

#include <lcm/lcm_coretypes.h>

#ifndef __ecat_data_t_hpp__
#define __ecat_data_t_hpp__



class ecat_data_t
{
    public:
        float      x[4];

        float      y[4];

        float      z[4];

        float      dx[4];

        float      dy[4];

        float      dz[4];

        float      fx[4];

        float      fy[4];

        float      fz[4];

        float      q_abad[4];

        float      q_hip[4];

        float      q_knee[4];

        float      dq_abad[4];

        float      dq_hip[4];

        float      dq_knee[4];

        float      tau_abad[4];

        float      tau_hip[4];

        float      tau_knee[4];

        float      tau_des_abad[4];

        float      tau_des_hip[4];

        float      tau_des_knee[4];

        int16_t    loop_count_ti[4];

        int16_t    ethercat_count_ti[4];

        int16_t    microtime_ti[4];

    public:
        /**
         * Encode a message into binary form.
         *
         * @param buf The output buffer.
         * @param offset Encoding starts at thie byte offset into @p buf.
         * @param maxlen Maximum number of bytes to write.  This should generally be
         *  equal to getEncodedSize().
         * @return The number of bytes encoded, or <0 on error.
         */
        inline int encode(void *buf, int offset, int maxlen) const;

        /**
         * Check how many bytes are required to encode this message.
         */
        inline int getEncodedSize() const;

        /**
         * Decode a message from binary form into this instance.
         *
         * @param buf The buffer containing the encoded message.
         * @param offset The byte offset into @p buf where the encoded message starts.
         * @param maxlen The maximum number of bytes to reqad while decoding.
         * @return The number of bytes decoded, or <0 if an error occured.
         */
        inline int decode(const void *buf, int offset, int maxlen);

        /**
         * Retrieve the 64-bit fingerprint identifying the structure of the message.
         * Note that the fingerprint is the same for all instances of the same
         * message type, and is a fingerprint on the message type definition, not on
         * the message contents.
         */
        inline static int64_t getHash();

        /**
         * Returns "ecat_data_t"
         */
        inline static const char* getTypeName();

        // LCM support functions. Users should not call these
        inline int _encodeNoHash(void *buf, int offset, int maxlen) const;
        inline int _getEncodedSizeNoHash() const;
        inline int _decodeNoHash(const void *buf, int offset, int maxlen);
        inline static uint64_t _computeHash(const __lcm_hash_ptr *p);
};

int ecat_data_t::encode(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;
    int64_t hash = (int64_t)getHash();

    tlen = __int64_t_encode_array(buf, offset + pos, maxlen - pos, &hash, 1);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = this->_encodeNoHash(buf, offset + pos, maxlen - pos);
    if (tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ecat_data_t::decode(const void *buf, int offset, int maxlen)
{
    int pos = 0, thislen;

    int64_t msg_hash;
    thislen = __int64_t_decode_array(buf, offset + pos, maxlen - pos, &msg_hash, 1);
    if (thislen < 0) return thislen; else pos += thislen;
    if (msg_hash != getHash()) return -1;

    thislen = this->_decodeNoHash(buf, offset + pos, maxlen - pos);
    if (thislen < 0) return thislen; else pos += thislen;

    return pos;
}

int ecat_data_t::getEncodedSize() const
{
    return 8 + _getEncodedSizeNoHash();
}

int64_t ecat_data_t::getHash()
{
    static int64_t hash = _computeHash(NULL);
    return hash;
}

const char* ecat_data_t::getTypeName()
{
    return "ecat_data_t";
}

int ecat_data_t::_encodeNoHash(void *buf, int offset, int maxlen) const
{
    int pos = 0, tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->x[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->y[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->z[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->dx[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->dy[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->dz[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fx[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fy[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->fz[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->q_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->q_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->q_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->dq_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->dq_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->dq_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_des_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_des_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_encode_array(buf, offset + pos, maxlen - pos, &this->tau_des_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->loop_count_ti[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->ethercat_count_ti[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_encode_array(buf, offset + pos, maxlen - pos, &this->microtime_ti[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ecat_data_t::_decodeNoHash(const void *buf, int offset, int maxlen)
{
    int pos = 0, tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->x[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->y[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->z[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->dx[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->dy[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->dz[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fx[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fy[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->fz[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->q_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->q_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->q_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->dq_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->dq_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->dq_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_des_abad[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_des_hip[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __float_decode_array(buf, offset + pos, maxlen - pos, &this->tau_des_knee[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->loop_count_ti[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->ethercat_count_ti[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    tlen = __int16_t_decode_array(buf, offset + pos, maxlen - pos, &this->microtime_ti[0], 4);
    if(tlen < 0) return tlen; else pos += tlen;

    return pos;
}

int ecat_data_t::_getEncodedSizeNoHash() const
{
    int enc_size = 0;
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __float_encoded_array_size(NULL, 4);
    enc_size += __int16_t_encoded_array_size(NULL, 4);
    enc_size += __int16_t_encoded_array_size(NULL, 4);
    enc_size += __int16_t_encoded_array_size(NULL, 4);
    return enc_size;
}

uint64_t ecat_data_t::_computeHash(const __lcm_hash_ptr *)
{
    uint64_t hash = 0x2dd37f039d5cbafcLL;
    return (hash<<1) + ((hash>>63)&1);
}

#endif
